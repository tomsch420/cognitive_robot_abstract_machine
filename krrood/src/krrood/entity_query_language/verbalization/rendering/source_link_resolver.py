from __future__ import annotations

import importlib
import inspect
import logging
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Protocol

from krrood.entity_query_language.verbalization.fragments.source_ref import SourceRef

_log = logging.getLogger(__name__)


class SourceLinkResolver(Protocol):
    """Maps a :class:`SourceRef` to a URL string, or ``None`` when unavailable."""

    def resolve(self, ref: SourceRef) -> Optional[str]:
        ...


@dataclass
class AutoAPIResolver:
    """Resolves source references to Sphinx AutoAPI documentation pages.

    *base_url* is the root of the generated docs site, e.g.
    ``https://myproject.readthedocs.io/en/latest`` or a local
    ``http://localhost:63342/project/doc/_build/html``.

    Use :meth:`for_package` to auto-detect the base URL for a locally installed
    package whose docs are served via the JetBrains IDE built-in HTTP server.

    When *html_root* is set (automatically populated by :meth:`for_package`),
    :meth:`resolve` checks that the generated AutoAPI page exists on disk and
    logs a ``WARNING`` if it does not — the class may be missing from the docs
    because they have not been built yet or because AutoAPI excluded it.
    """

    base_url: str
    html_root: Optional[Path] = None

    def resolve(self, ref: SourceRef) -> Optional[str]:
        try:
            module = ref.cls.__module__
            qualname = ref.cls.__qualname__
        except AttributeError:
            return None
        module_path = module.replace(".", "/")
        anchor = f"{module}.{qualname}"
        if ref.attribute is not None:
            anchor = f"{anchor}.{ref.attribute}"
        base = self.base_url.rstrip("/")
        url = f"{base}/autoapi/{module_path}/index.html#{anchor}"
        if self.html_root is not None:
            page = self.html_root / "autoapi" / module_path / "index.html"
            if not page.exists():
                _log.warning(
                    "AutoAPI page for %s.%s does not exist at %s — "
                    "the class may be missing from the docs; "
                    "try rebuilding: sphinx-build doc doc/_build/html",
                    module,
                    qualname,
                    page,
                )
        return url

    @classmethod
    def for_package(cls, package_name: str, port: int = 63342) -> "AutoAPIResolver":
        """Build an :class:`AutoAPIResolver` for *package_name*'s locally built Sphinx docs.

        The base URL targets the JetBrains IDE built-in HTTP server using this algorithm:

        1. Import *package_name* to locate its source tree.
        2. Walk up to the directory containing ``pyproject.toml`` (the package root).
        3. Expect the Sphinx HTML output at ``{package_root}/doc/_build/html``.
        4. Walk up to the git root (directory containing ``.git``).
        5. Construct ``http://localhost:{port}/{git_root_name}/{relative_html_path}``.

        :raises ImportError: if *package_name* cannot be imported.
        :raises FileNotFoundError: if ``doc/_build/html`` does not exist —
            build the docs first with ``sphinx-build doc doc/_build/html``.
        """
        try:
            pkg = importlib.import_module(package_name)
        except ImportError as exc:
            raise ImportError(f"Cannot import package {package_name!r}: {exc}") from exc

        pkg_file = Path(inspect.getfile(pkg)).resolve()

        package_root: Optional[Path] = None
        for parent in pkg_file.parents:
            if (parent / "pyproject.toml").exists():
                package_root = parent
                break
        if package_root is None:
            raise FileNotFoundError(f"No pyproject.toml found in any parent of {pkg_file}")

        html_root = package_root / "doc" / "_build" / "html"
        if not html_root.exists():
            raise FileNotFoundError(
                f"Sphinx HTML output not found at {html_root}. "
                f"Build the docs first: sphinx-build doc doc/_build/html"
            )

        git_root: Optional[Path] = None
        for parent in [package_root, *package_root.parents]:
            if (parent / ".git").exists():
                git_root = parent
                break
        if git_root is None:
            git_root = package_root.parent

        rel_html = html_root.relative_to(git_root)
        return cls(
            base_url=f"http://localhost:{port}/{git_root.name}/{rel_html}",
            html_root=html_root,
        )
