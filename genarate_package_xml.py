# Generate package.xml

import os, sys, argparse

CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
TEMPLATE_FILE = os.path.join(CURRENT_DIR, 'package.xml')

def generate_string(name):
    with open(TEMPLATE_FILE) as f:
        return f.read().format(name=name)

def strip_trailing_slash(s):
    while s.endswith(os.path.sep):
        s = s[:-1]
    return s

def generate_one(path):
    dst = os.path.join(path, 'package.xml')
    if os.path.isfile(dst):
        print('{} exists'.format(dst))
        return
    path = strip_trailing_slash(path)
    name = os.path.basename(path)
    xml = generate_string(name)
    with open(dst, 'w') as f:
        f.write(xml)

def generate_from_manifest(path):
    import yaml
    with open(path) as f:
        doc = yaml.load(f)
    repositories = doc['repositories']
    for repo in repositories:
        package = os.path.join(os.path.dirname(path), 'src', repo)
        generate_one(package)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('path', help='package/manifest path')
    args = parser.parse_args()

    path = args.path
    if os.path.isfile(path):
        generate_from_manifest(path)
    else:
        generate_one(path)

if __name__ == '__main__':
    main()
