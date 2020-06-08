import argparse
import struct


def main(args):
    data_string = f'{len(args.pattern)}{args.data_type}'
    pack_string = f'{args.byte_order}' + data_string * args.size
    packed_data = struct.pack(pack_string, *(args.pattern * args.size))
    with open(args.output, 'wb') as f:
        f.write(packed_data)


def auto_int(x):
    return int(x, 0)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Binary pattern generator')
    parser.add_argument('-p', type=auto_int, nargs='+', required=True, action="store", dest="pattern", help="Fill pattern.")
    parser.add_argument('-d', default='B', action="store", dest="data_type", help="Data type to use for 'pattern'. B or I.")
    parser.add_argument('-s', type=auto_int, required=True, action="store", dest="size", help="File size in multiples of fill pattern/data_type.")
    parser.add_argument('-e', default='<', action="store", dest="byte_order", help="Byte order (endianess). le: <, be: >.")
    parser.add_argument('-o', required=True, action="store", dest="output", help="Path to save output.")

    main(parser.parse_args())
