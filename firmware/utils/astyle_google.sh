#!/bin/bash
# This script is a wrapper on top of AStyle to beautify the code to comply
# with Google coding style (as close as possible).
#
# If the source_path is a file, AStyle it.
# If the source_path is a directory, AStyle selected file types (.h, .hpp, .cc, .cpp)
# in source_path recursrively.  Use with CAUTION!!!
if [ $# -ne 1 ]; then
    echo " Usage:"
    echo " (single file):                     astyle_google.sh /project/path/filename"
    echo " (recursive on .h/.hpp/.cc/.cpp)  : astyle_google.sh /project/path"
    exit
fi
source_path=$1

# Locate AStyle binary
#astyle_bin=~/Development/astyle/build/mac/bin/AStyle
astyle_bin=/usr/bin/astyle
if [[ ! -f ${astyle_bin} ]]; then
    echo "${astyle_bin} doesn't exist!"
    echo " Download AStyle @ http://astyle.sourceforge.net"
    exit
fi

# AStyle options to comply with our desired google c++ coding style
# Check http://astyle.sourceforge.net/astyle.html for details of these options
style_options="--style=google \
               --indent=spaces=2 \
               --break-after-logical \
               --indent-modifiers \
               --suffix=none \
               --keep-one-line-blocks \
               --keep-one-line-statements \
               --pad-header \
               --pad-oper \
               --pad-comma \
               --convert-tabs"
             # --max-continuation-indent=100"
             # --max-code-length=100 \

if [[ -d ${source_path} ]]; then
    # Find files whose names matches the regex pattern(s)
    # The --recursive seems not workgin properly on OSX,
    # so we use the old-school way to find each file and AStyle it
    files=`find -E ${source_path} -regex '.*\.(cpp|cc|hpp|h)' -print0 | xargs -0`
    for file in $files
    do
        ${astyle_bin} ${style_options} $file
    done
elif [[ -f ${source_path} ]]; then
    ${astyle_bin} ${style_options} ${source_path}
else
    echo "invalid ${source_path}"
fi
