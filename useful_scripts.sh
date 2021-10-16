forall() {
	COMMAND=$@
	if [ ${#COMMAND[@]} -le 1 ]; then
    	echo "No command entered! Please enter a command to run!"
		return;
	fi
	echo "Running $COMMAND"
	find . -maxdepth 1 \( ! -name . \) | grep -v \\./\\. | xargs -I {} bash -c "cd '{}' && echo && echo {} && $COMMAND";
}
