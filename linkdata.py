#
# Data Linker
# Author:  Jared Sanson <jared@jared.geek.nz>
# License: MIT
#

import json
import re
import os.path
from glob import glob
from typing import Type
from SCons import *
from LVGLImage import ColorFormat, OutputFormat, CompressMethod, PNGConverter, LVGLImage

target_h = 'Data.h'
target_h_path = 'src/' + target_h

data_src = glob('data/*')
# img_src = [
#     f for f in img_src if 
#     os.path.basename(os.path.dirname(f)) != 'src'
# ]

Import("env")

env['TOOLCHAINPREFIX'] = 'xtensa-esp32-elf-'

# Do not run extra script when IDE fetches C/C++ project metadata
from SCons.Script import COMMAND_LINE_TARGETS
if "idedata" in COMMAND_LINE_TARGETS:
    env.Exit(0)

def symbol_name(path):
    #return os.path.splitext(os.path.basename(path))[0].replace('-','_')
    return os.path.basename(path).replace('-','_').replace('.','_')

def gen_data_header(source, target, env):
    def emit_lines():
        yield '// Automatically generated'
        yield '#pragma once'
        yield ''
        yield '#include "DataFile.h"'
        yield ''
        yield f'// Raw symbol references'
        for src in source:
            symname = symbol_name(src.get_path())
            yield f'extern const uint8_t _binary_data_{symname}_start[];'
            yield f'extern const uint8_t _binary_data_{symname}_end[];'
            yield f'extern const size_t _binary_data_{symname}_size;'

        yield ''
        yield 'namespace data {'
        for src in source:
            symname = symbol_name(src.get_path())
            yield f'  // {src}'
            yield f'  static const uint8_t* {symname}_bytes = _binary_data_{symname}_start;'
            yield f'  static const size_t {symname}_size = (size_t)(_binary_data_{symname}_end - _binary_data_{symname}_start);'
            #yield f'  static const size_t {symname}_size = _binary_data_{symname}_size;'
            #yield f'  static DataStream {symname} {{  _binary_data_{symname}_start, _binary_data_{symname}_size }};'
            yield f''
        yield '}'

    for t in target:
        with open(t.get_path(), 'wb') as f:
            for ln in emit_lines():
                f.write(bytes(ln + '\n', 'utf-8'))


# Ensure header is generated before attempting to compile source
# by marking it as a dependency for any files that include the header.
src = env.FindSourceFiles()
src = [d for d in src if d.get_path().startswith('src')] # Limit paths to src/
src = [d for d in src if ('"',target_h,'"') in d.includes]
for d in src:
    env.Depends(
        target=f"$BUILD_DIR/{d.get_path()}.o",
        dependency=[target_h]
    )


targets = []
for file in data_src:
    # The source is the raw file, the target will be the .o object file
    #data_o = file.rsplit('.', 1)[0] + '.o'  # Generate the corresponding .o file name
    fname = file.rsplit('.', 1)[0]
    symname = symbol_name(file)
    data_o = f'$BUILD_DIR/data/{fname}.o'

    print(f'Linking Data {file}')

    # Create a custom builder for linking raw files into object files
    env.Command(
        target=data_o,
        source=file,
        action=(f'xtensa-esp32-elf-objcopy --input-target=binary --output-target=elf32-xtensa-le --binary-architecture=xtensa --rename-section .data=.rodata.{symname} $SOURCE $TARGET')
    )
    
    targets.append(data_o)

# Generate a common header
env.Command(
    target=[target_h_path],
    source=data_src,
    action=env.Action(gen_data_header, f"Generate {target_h}"),
    source_scanner=None
)

# Create a library for linking
datalib = env.StaticLibrary(
    '$BUILD_DIR/Data',
    targets
)
env.Append(LIBS=[datalib])
#env.Append(LINKFLAGS=["-mno-check-zero-architecture"])
#–accept-unknown-input-arch 

env.Append(CPATH=['DataFile.h'])

env.NoClean(data_src)
