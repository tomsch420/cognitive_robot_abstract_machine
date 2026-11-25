"""
Latent variable distillation of :cite:`liu2023scaling` for hidden Chow-Liu tree
circuits: a masked autoencoder trained on the data assigns every latent variable by
clustering the embedding of its variable, and the circuit is counted from the completed
rows.
"""

from __future__ import annotations

# %% imports
from dataclasses import dataclass

import numpy as np
import torch
from random_events.variable import Symbolic
from sklearn.cluster import MiniBatchKMeans
from sortedcontainers import SortedSet

from probabilistic_model.learning.layered.hidden_chow_liu_tree import (
    HiddenChowLiuTree,
)
from probabilistic_model.probabilistic_circuit.tensorized.layered_probabilistic_circuit import (
    LayeredProbabilisticCircuit,
)
from probabilistic_model.probabilistic_circuit.tensorized.symbolic_encoding import (
    SymbolicEncoding,
)


# %% masked autoencoder


class TabularMaskedAutoencoder(torch.nn.Module):
    """
    A masked autoencoder :cite:`liu2023scaling` over the variables of a table: every
    variable is one token, a transformer encodes the visible tokens and the masked ones,
    and one head per variable reconstructs its value.
    """

    def __init__(
        self, *, number_of_states: list[int], width: int, layers: int, heads: int
    ):
        """
        :param number_of_states: The number of states of every symbolic variable, zero
            for a continuous one.
        :param width: The size of a token embedding.
        :param layers: The number of transformer layers.
        :param heads: The number of attention heads.
        """
        super().__init__()
        self.number_of_states = number_of_states
        self.value_embeddings = torch.nn.ModuleList(
            torch.nn.Embedding(states, width) if states else torch.nn.Linear(1, width)
            for states in number_of_states
        )
        self.position_embedding = torch.nn.Parameter(
            torch.randn(len(number_of_states), width) * 0.02
        )
        self.mask_embedding = torch.nn.Parameter(torch.zeros(width))
        self.encoder = torch.nn.TransformerEncoder(
            torch.nn.TransformerEncoderLayer(
                d_model=width, nhead=heads, dim_feedforward=2 * width, batch_first=True
            ),
            num_layers=layers,
        )
        self.heads = torch.nn.ModuleList(
            torch.nn.Linear(width, states if states else 1)
            for states in number_of_states
        )

    def embeddings(self, *, values: torch.Tensor, masked: torch.Tensor) -> torch.Tensor:
        """
        :param values: The state index or the value of every variable, shape (#rows,
            #variables).
        :param masked: Whether every value is hidden from the encoder.
        :return: The encoded token of every variable, shape (#rows, #variables, width).
        """
        tokens = torch.stack(
            [
                embedding(values[:, index].long())
                if states
                else embedding(values[:, index : index + 1])
                for index, (embedding, states) in enumerate(
                    zip(self.value_embeddings, self.number_of_states)
                )
            ],
            dim=1,
        )
        tokens = torch.where(masked[..., None], self.mask_embedding, tokens)
        return self.encoder(tokens + self.position_embedding)

    def reconstruction_loss(
        self, *, values: torch.Tensor, masked: torch.Tensor
    ) -> torch.Tensor:
        """
        :param values: The state index or the value of every variable.
        :param masked: Whether every value is hidden from the encoder.
        :return: The average loss of reconstructing the hidden values: cross entropy for
            symbolic and squared error for continuous variables.
        """
        encoded = self.embeddings(values=values, masked=masked)
        losses = []
        for index, (head, states) in enumerate(zip(self.heads, self.number_of_states)):
            prediction = head(encoded[:, index])
            if states:
                loss = torch.nn.functional.cross_entropy(
                    prediction, values[:, index].long(), reduction="none"
                )
            else:
                loss = (prediction[:, 0] - values[:, index]) ** 2
            losses.append((loss * masked[:, index]).sum())
        return torch.stack(losses).sum() / masked.sum().clamp(min=1)


# %% latent variable distillation


@dataclass(kw_only=True)
class LatentVariableDistillation:
    """
    Learns a hidden Chow-Liu tree circuit whose latent variables are first assigned by a
    masked autoencoder trained on the same rows.
    """

    variables: SortedSet
    """
    The variables, one per column of the data.
    """

    hidden_states: int = 12
    """
    The number of states of every latent variable, the number of clusters of every
    variable's embeddings.
    """

    width: int = 64
    """
    The size of a token embedding of the autoencoder.
    """

    layers: int = 2
    """
    The number of transformer layers of the autoencoder.
    """

    heads: int = 4
    """
    The number of attention heads of the autoencoder.
    """

    mask_probability: float = 0.5
    """
    The probability that a value is hidden from the encoder during training.
    """

    epochs: int = 10
    """
    The number of passes of the autoencoder over the data.
    """

    batch_size: int = 256
    """
    The number of rows per step of the autoencoder.
    """

    learning_rate: float = 1e-3
    """
    The learning rate of the autoencoder's Adam optimizer.
    """

    seed: int = 0
    """
    The seed of the autoencoder and the clustering.
    """

    def build_circuit(self, *, data: np.ndarray) -> LayeredProbabilisticCircuit:
        """
        :param data: The rows, whose symbolic values are hashes of domain elements.
        :return: The hidden Chow-Liu tree circuit whose parameters are counted with the
            latent states the autoencoder assigns, with jax parameters.
        """
        torch.manual_seed(self.seed)
        device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        number_of_states = [
            len(variable.domain.simple_sets) if isinstance(variable, Symbolic) else 0
            for variable in self.variables
        ]
        values = torch.as_tensor(
            self.autoencoder_values(data=data), dtype=torch.float32, device=device
        )
        model = TabularMaskedAutoencoder(
            number_of_states=number_of_states,
            width=self.width,
            layers=self.layers,
            heads=self.heads,
        ).to(device)
        self.train_autoencoder(model=model, values=values)
        latent_states = self.latent_states(model=model, values=values)
        return HiddenChowLiuTree(
            variables=self.variables, hidden_states=self.hidden_states, seed=self.seed
        ).build_circuit(data=data, latent_states=latent_states)

    def autoencoder_values(self, *, data: np.ndarray) -> np.ndarray:
        """
        :param data: The rows, whose symbolic values are hashes.
        :return: The rows with every symbolic value replaced by its state index.
        """
        values = np.array(data, dtype=float)
        for index, variable in enumerate(self.variables):
            if isinstance(variable, Symbolic):
                values[:, index] = SymbolicEncoding(variable).indices_of_hashes(
                    data[:, index]
                )
        return values

    def train_autoencoder(
        self, *, model: TabularMaskedAutoencoder, values: torch.Tensor
    ) -> None:
        """
        :param model: The autoencoder, trained in place.
        :param values: The state index or value of every variable of every row.
        """
        optimizer = torch.optim.Adam(model.parameters(), lr=self.learning_rate)
        generator = torch.Generator(device=values.device).manual_seed(self.seed)
        model.train()
        for _ in range(self.epochs):
            order = torch.randperm(
                len(values), device=values.device, generator=generator
            )
            for start in range(0, len(values), self.batch_size):
                batch = values[order[start : start + self.batch_size]]
                masked = (
                    torch.rand(batch.shape, device=values.device, generator=generator)
                    < self.mask_probability
                )
                optimizer.zero_grad()
                model.reconstruction_loss(values=batch, masked=masked).backward()
                optimizer.step()

    def latent_states(
        self, *, model: TabularMaskedAutoencoder, values: torch.Tensor
    ) -> np.ndarray:
        """
        :param model: The trained autoencoder.
        :param values: The state index or value of every variable of every row.
        :return: The cluster of the embedding of every variable in every row, with all
            values visible, shape (#rows, #variables).
        """
        model.eval()
        embeddings = []
        with torch.no_grad():
            for start in range(0, len(values), self.batch_size):
                batch = values[start : start + self.batch_size]
                embeddings.append(
                    model.embeddings(
                        values=batch, masked=torch.zeros_like(batch, dtype=torch.bool)
                    )
                    .cpu()
                    .numpy()
                )
        embeddings = np.concatenate(embeddings)
        return np.stack(
            [
                MiniBatchKMeans(
                    n_clusters=self.hidden_states, random_state=self.seed, n_init=3
                ).fit_predict(embeddings[:, index])
                for index in range(len(self.variables))
            ],
            axis=1,
        )
