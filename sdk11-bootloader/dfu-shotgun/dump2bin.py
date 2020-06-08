import struct
import argparse


def main(args):
    vals = []
    with open(args.input_file, 'rt') as fIn:
            for line in fIn:
                val = int(line, 16)
                vals.append(val)

    pack_string = f'<{len(vals)}I'
    packed_data = struct.pack(pack_string, *vals)
    with open(args.output_file, 'wb') as fOut:
        fOut.write(packed_data)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Convert hex dump to binary')
    parser.add_argument('-i', required=True, action="store", dest="input_file", help="Input text file of 32b hex value per line.")
    parser.add_argument('-o', required=True, action="store", dest="output_file", help="Output binary file.")

    main(parser.parse_args())
