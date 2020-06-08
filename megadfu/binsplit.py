import os
import argparse


def main(args):
    sizeA = os.stat(args.partition_file).st_size + args.offset
    sizeB = os.stat(args.input_file).st_size - sizeA

    with open(args.input_file, 'rb') as fIn:
        chunkA = fIn.read(sizeA)
        chunkB = fIn.read(sizeB)
        if (args.output_pA):
            with open(args.output_pA, 'wb') as fOutA:
                fOutA.write(chunkA)
        if (args.output_pB):
            with open(args.output_pB, 'wb') as fOutB:
                fOutB.write(chunkB)


def auto_int(x):
    return int(x, 0)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Binary splitter')
    parser.add_argument('-i', required=True, action="store", dest="input_file", help="Input file to split.")
    parser.add_argument('-partition', required=True, action="store", dest="partition_file", help="Input file to use as a partition boundary for the split.")
    parser.add_argument('-offset', type=auto_int, default=0, action="store", dest="offset", help="Offset from the partition filesize to split at.")
    parser.add_argument('-pA', action="store", dest="output_pA", help="Output file for part A.")
    parser.add_argument('-pB', action="store", dest="output_pB", help="Output file for part B.")

    main(parser.parse_args())
