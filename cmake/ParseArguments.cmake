# Parse arguments passed to a function into several lists separated by
# upper-case identifiers and options that do not have an associated list e.g.:
#
# SET(arguments
#   hello OPTION3 world
#   LIST3 foo bar
#   OPTION2
#   LIST1 fuz baz
#   )
# PARSE_ARGUMENTS(ARG "LIST1;LIST2;LIST3" "OPTION1;OPTION2;OPTION3" ${arguments})
#
# results in 7 distinct variables:
#  * ARG_DEFAULT_ARGS: hello;world
#  * ARG_LIST1: fuz;baz
#  * ARG_LIST2:
#  * ARG_LIST3: foo;bar
#  * ARG_OPTION1: FALSE
#  * ARG_OPTION2: TRUE
#  * ARG_OPTION3: TRUE
#
# taken from http://www.cmake.org/Wiki/CMakeMacroParseArguments 

MACRO(PARSE_ARGUMENTS prefix arg_names option_names)
    SET(DEFAULT_ARGS)
    FOREACH(arg_name ${arg_names})    
        SET(${prefix}_${arg_name})
    ENDFOREACH(arg_name)
    FOREACH(option ${option_names})
        SET(${prefix}_${option} FALSE)
    ENDFOREACH(option)
    
    SET(current_arg_name DEFAULT_ARGS)
    SET(current_arg_list)
    FOREACH(arg ${ARGN})            
        SET(larg_names ${arg_names})    
        LIST(FIND larg_names "${arg}" is_arg_name)                   
        IF (is_arg_name GREATER -1)
            SET(${prefix}_${current_arg_name} ${current_arg_list})
            SET(current_arg_name ${arg})
            SET(current_arg_list)
        ELSE (is_arg_name GREATER -1)
            SET(loption_names ${option_names})    
            LIST(FIND loption_names "${arg}" is_option)            
            IF (is_option GREATER -1)
                SET(${prefix}_${arg} TRUE)
            ELSE (is_option GREATER -1)
                SET(current_arg_list ${current_arg_list} ${arg})
            ENDIF (is_option GREATER -1)
        ENDIF (is_arg_name GREATER -1)
    ENDFOREACH(arg)
    SET(${prefix}_${current_arg_name} ${current_arg_list})
ENDMACRO(PARSE_ARGUMENTS)
