import argparse
import struct

def main(args):
    inputs = []
    descriptor = []
    for input in args.inputs:
        startOffset = auto_int(input[1])
        size = auto_int(input[2])
        alignedSize = (size + args.alignment - 1) & ~(args.alignment - 1)
        i = {
            "filename": input[0],
            "startOffset": startOffset,
            "endOffset": startOffset+size,
            "size": size,
            "alignedSize": alignedSize,
        }
        inputs.append(i)
        descriptor.append(alignedSize)

    with open(args.outputBinary, 'wb') as fOut:
        for input in inputs:
            with open(input["filename"], 'rb') as fIn:
                fIn.seek(input["startOffset"])
                chunk = fIn.read(input["size"])
                
                sizeDiff = input["alignedSize"] - input["size"]
                alignedChunk = chunk + (bytearray(b'\x00') * sizeDiff)
                
                fOut.write(alignedChunk)

    pack_string = f'I' * len(inputs)
    packed_data = struct.pack(pack_string, *descriptor)
    with open(args.outputDescriptor, 'wb') as fDesc:
        fDesc.write(packed_data)


def auto_int(x):
    return int(x, 0)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Binary merger')
    parser.add_argument('-i', required=True, action="append", nargs=3, metavar=("filename", "offset", "size"), dest="inputs", help="Input binary to merge.")
    parser.add_argument('-o', required=True, action="store", dest="outputBinary", help="Path to save output.")
    parser.add_argument('-d', required=True, action="store", dest="outputDescriptor", help="Path to save output descriptor.")
    parser.add_argument('-a', type=auto_int, default=0x1000, required=False, action="store", dest="alignment", help="Alignment of binary chunks.")

    main(parser.parse_args())
