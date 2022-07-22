#######################mkrobot.sh################################
_mkrobot_completions()
{
	if [ "${#COMP_WORDS[@]}" != "2" ]; then
		return;
	fi
	COMPREPLY=($(compgen -W "build clone clean cleanlibs cleanros commit deploy node push rebuild rebuildlibs rebuildros tag test update launch" "${COMP_WORDS[1]}"))
}

complete -F _mkrobot_completions mkrobot.sh
#######################mkrobot.sh################################
