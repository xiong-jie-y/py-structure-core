import click
import shutil
import os
from os.path import join
import glob
import re

@click.command()
@click.option("--sdk-path")
def install_sdk(sdk_path):
    shutil.copytree(join(sdk_path, 'Libraries/Structure'), join(os.getcwd(), 'structure_core'))
    shutil.move(join(os.getcwd(), "structure_core/Headers/ST"), join(os.getcwd(), 'ST'))

    for header_file in glob.glob(join(os.getcwd(), 'ST/*.h')):
        new_content_lines = []
        for line in open(header_file).readlines():
            new_content_lines.append(re.sub(r'#include <ST/(.*)>', r'#include "ST/\1"', line))
        open(header_file, "w").writelines(new_content_lines)

if __name__ == '__main__':
    install_sdk()