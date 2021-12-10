#######################mkrobot.sh################################
_mkrobot_completions()
{
	if [ "${#COMP_WORDS[@]}" != "2" ]; then
		return;
	fi
	COMPREPLY=($(compgen -W "build clone clean cleanlibs cleanros node rebuild rebuildlibs rebuildros test update" "${COMP_WORDS[1]}"))
}

complete -F _mkrobot_completions mkrobot.sh
#######################mkrobot.sh################################