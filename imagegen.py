#
# Image Generator using LVGL image format
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

target_h = 'Images.h'
target_h_path = 'src/' + target_h
lvgl_h_path = 'src/lvgl/lvgl.h'

img_src = glob('img/*/*.png')

img_typedef = 'const lv_image_dsc_t'

default_cf = ColorFormat.A1  # Alpha 1bit

Import("env")

# Do not run extra script when IDE fetches C/C++ project metadata
from SCons.Script import COMMAND_LINE_TARGETS
if "idedata" in COMMAND_LINE_TARGETS:
    env.Exit(0)

def imagename(path):
    return os.path.splitext(os.path.basename(path))[0].replace('-','_')

def generate_image(source, target, env):
    src_path = source[0].get_path() # .png
    dst_path = target[0].get_path() # .c
    img_name = imagename(src_path)

    # Allow subfolder to specify color format
    cf = default_cf
    cf_str = os.path.basename(os.path.dirname(src_path))
    if cf_str != 'img':
        cf = ColorFormat[cf_str]

    print(f'Generating {img_name} ({cf})')

    if cf.is_indexed:
        raise NotImplementedError('indexed color formats not supported')
    
    img = LVGLImage().from_png(src_path, cf)
    #img.adjust_stride(align=1)
    #img.premultiply()
    img.to_c_array(dst_path, compress=CompressMethod.NONE)

    # converter = PNGConverter(
    #     [src_path],
    #     cf,
    #     ofmt,
    #     dst_path,
    #     compress=CompressMethod.NONE,
    #     keep_folder=False
    # )

    #output = converter.convert()
    #for f, img in output:
    #    print(f'  {img.w}x{img.h}, {img.data_len} bytes')


def gen_image_header(source, target, env):
    def emit_lines():
        yield '// Automatically generated'
        yield '// Format: LVGLv8, CCF_ALPHA_1BIT'
        yield '#pragma once'
        yield ''
        yield '#include "lvgl/lvgl.h"'
        yield ''
        #yield 'namespace img {'
        #yield '  extern "C" {'
        for src in source:
            img_name = imagename(src.get_path())
            #yield f'    extern {img_typedef} {img_name};'
            yield f'extern {img_typedef} {img_name};'
        #yield '  }'
        #yield '}'

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
for img in img_src:
    name = os.path.splitext(os.path.basename(img))[0]
    img_c = f'$BUILD_DIR/img/{name}.c'
    img_o = f'$BUILD_DIR/img/{name}.o'

    # Compile images to CPP files
    env.Command(
        target=[img_c],
        source=[img],
        action=env.Action(generate_image, "Generate Image"),
        source_scanner=None
    )

    # Compile .cpp to .o
    env.Object(target=img_o, source=img_c)
    targets.append(img_o)

# Generate a common header
env.Command(
    target=[target_h_path],
    source=img_src,
    action=env.Action(gen_image_header, f"Generate {target_h}"),
    source_scanner=None
)

# Create a library for linking
imglib = env.StaticLibrary(
    '$BUILD_DIR/Images',
    targets
)
env.Append(LIBS=[imglib])

env.Append(CPATH=[lvgl_h_path])

env.NoClean(img_src)
