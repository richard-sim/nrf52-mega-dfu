import os
import argparse
import struct


def main(args):
    packed_data = struct.pack("I" * len(args.values), *args.values)
    with open(args.output_file, 'wb') as fOut:
        fOut.write(packed_data)


def auto_int(x):
    return int(x, 0)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Binary splitter')
    parser.add_argument('-v', type=auto_int, nargs='+', required=True, action="store", dest="values", help="Values for struct elements.")
    parser.add_argument('-o', required=True, action="store", dest="output_file", help="File to write generated struct to.")

    main(parser.parse_args())
