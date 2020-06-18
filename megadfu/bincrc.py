import argparse
import binascii


def calculate_crc(firmware_filename):
    """
    Calculates CRC16 has on provided firmware filename
    :type str firmware_filename:
    """
    data_buffer = b''
    read_size = 4096
    
    crc = 0

    with open(firmware_filename, 'rb') as firmware_file:
        while True:
            data = firmware_file.read(read_size)

            if data:
                data_buffer += data
                crc = binascii.crc32(data, crc)
            else:
                break

    return (binascii.crc32(data_buffer), crc)


def main(args):
    crc, running_crc = calculate_crc(args.input)
    print(f'CRC32: {crc:08x}. Running CRC: {running_crc:08x}')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='CRC32 calculator')
    parser.add_argument('-i', required=True, action="store", dest="input", help="Path to input file.")

    main(parser.parse_args())
