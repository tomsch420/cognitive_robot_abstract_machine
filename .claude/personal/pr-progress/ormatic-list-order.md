Branch `ormatic-list-order`, stacked on PR cram2#1065 (`ormatic-eql-result-loading`); fixes cram2#1026.
Plan file: ~/.claude/plans/snug-hopping-cosmos-of-a143fab8-dc42-4477-b03e-a5b96e7394fd.md

Draft PR: tomsch420/cognitive_robot_abstract_machine#58 (base ormatic-eql-result-loading), label bug.

Plan:
- (done) index_/key_ columns + order_by, nested collections, dicts, DictJSONSerializer

Done:
- fde30cec7f pushed; ormatic 371, utils+class_diagram 203, eql 1645, segmind/cramph orm green locally
- schema diff of workspace interfaces listed in PR description

Next:
- giskardpy tests (need rclpy) and coraplex/experiments generation (pre-existing DetectAction failure) only in CI
- labels api-break/new-feature/bug-fix missing on fork: set them upstream when promoting
- ask: remove now unused WrappedField.is_collection_of_builtins / is_collection_of_types?
