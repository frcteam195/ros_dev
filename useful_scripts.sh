forall() {
	COMMAND=$@
	echo "Running $COMMAND"
	find . -maxdepth 1 \( ! -name . \) | grep -v \\./\\. | xargs -I {} bash -c "cd '{}' && echo && echo {} && $COMMAND";
}
