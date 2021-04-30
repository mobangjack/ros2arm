# Generate mirror manifest.

import os, argparse

CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
MIRROR = "https://gitee.com/mobangjack"

def strip_trailing_slash(s):
    while s.endswith(os.path.sep):
        s = s[:-1]
    return s

def get_repo_name(url):
    url = strip_trailing_slash(url)
    dummy = url.split('/')
    return dummy[-1]

def make_mirror_manifest(input, output, mirror):
    import yaml
    with open(input) as f:
        doc = yaml.load(f)
    repositories = doc['repositories']
    for item in repositories:
        repo = repositories[item]
        origin_url = repo['url']
        name = get_repo_name(origin_url)
        mirror_url = mirror + "/" + name
        repo['url'] = mirror_url
    with open(output, 'w') as f:
        yaml.dump(doc, f, indent=2, default_flow_style=False)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('-i', '--input', required=True,
                        help='input manifest file path')
    parser.add_argument('-o', '--output', required=True,
                        help='output manifest file path')
    parser.add_argument('-m', '--mirror', default=MIRROR,
                        help='mirror base url')
    args = parser.parse_args()

    make_mirror_manifest(args.input, args.output, args.mirror)

if __name__ == '__main__':
    main()
