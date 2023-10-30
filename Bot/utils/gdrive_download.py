import argparse
import gdown
parser = argparse.ArgumentParser()


parser.add_argument('--link_drive', help='link to driver folder', type=str)
parser.add_argument('--dest_path', help='destination path', type=str)
args = parser.parse_args()

if __name__ == 'main':
    base_string = "https://drive.google.com/uc?id="
    print(f"Download from {base_string+args.link_drive} - Destination {args.dest_path}")
    gdown.download(base_string+args.link_drive, args.dest_path, quiet=False, use_cookies=False, verify=False)