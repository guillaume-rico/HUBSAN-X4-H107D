#!/usr/bin/python

import csv
import sys
import array
import a7105
import argparse

# Original work of Avalrop : https://github.com/alvarop/protox
# V2 : Allow openbench files 
# use : python quadcsv.py -r rawdata\handset_export_4.2V.csv -a openbench
# C:\Python27\python.exe D:\Perso\HUBSAN-X4-H107D\reverse\A7105\quadcsv.py -r D:\Perso\HUBSAN-X4-H107D\reverse\A7105\rawdata\handset_start.csv
# From Logic sniffer, tools, SPI Analyser, you must check "SHOW /CS?"

# Use this script to process Saleae Logic captured SPI data (from CSV) to more manageable chunks
# 
# I added duplicate packet detection so we don't print the same thing over and over
# I've also added packet decoding, so you get some human readable stuff
# Check out the a7105.py file for details on the decoding
# 
# Here are some examples running the script
# For a single capture and no decoding:
# $ python quadcsv.py -r rawdata/connect2.csv
# 
# or for a dual-capture with decoding:
# $ python ./quadcsv.py -r rawdata/dual-capture-remote.csv -q rawdata/dual-capture-quad.csv -d
# 
# 
# Input file looks like this:
# 4.406020080000000,41055,0x40,
# 4.406054800000000,41055,0x19,
# 4.406096400000000,41056,0x40,
# 4.406135600000000,41056,0x19,
# 4.406178720000000,41057,0xA0,
# 4.406216720000000,41058,0xE0,
# 4.406253680000000,41059,0x05,
# 4.406290560000000,41059,0x01,
# 4.406327600000000,41059,0x46,
# 4.406364720000000,41059,0xE4,
# 4.406401760000000,41059,0x59,
# 4.406438880000000,41059,0x10,
# 4.406476000000000,41059,0x3F,
# 4.406513040000000,41059,0x00,
# 4.406550160000000,41059,0x00,
# 4.406587280000000,41059,0x00,
# 4.406624320000000,41059,0x00,
# 4.406661440000000,41059,0x00,
# 4.406698560000000,41059,0x00,
# 4.406735680000000,41059,0x00,
# 4.406772720000000,41059,0x00,
# 4.406809840000000,41059,0x00,
# 4.406847040000000,41059,0x2D,
# 4.406884320000000,41060,0xD0,
# 4.406921760000000,41061,0x40,
# 4.406956480000000,41061,0x1B,
# 4.406995680000000,41062,0x40,
# 4.407030400000000,41062,0x1B,
# 
# Processed output looks like:
# 40 19  (Repeated 148 times)
# a0 
# e0 
# 05 01 46 e4 59 10 3f 00 00 00 00 00 00 00 00 00 2d 
# d0 
# 40 1b  (Repeated 26 times)
# 40 1a 
# c0 
# 40 19  (Repeated 148 times)
#

# Decoded output looks like:
# remote 3.621992 R MODE: FECOK  CRCOK  RFEN  XEN  PLLDIS TRXDIS TX  (Repeated 148 times)
# remote 3.622030 S Standby Mode
# remote 3.622067 S FIFO Write pointer reset
# remote 3.622698 W FIFO [01 82 C0 AC D8 41 00 00 00 00 00 00 00 00 00 F8]
# remote 3.622736 S TX Mode
# remote 3.624740 R MODE: FECOK  CRCOK  RFEN  XEN  PLLDIS TRXEN  TX  (Repeated 26 times)
# remote 3.624814 R MODE: FECOK  CRCOK  RFEN  XEN  PLLDIS TRXEN  RX 
# remote 3.624854 S RX Mode
# remote 3.636266 R MODE: FECOK  CRCOK  RFEN  XEN  PLLDIS TRXDIS TX  (Repeated 148 times)


# 
# Open CSV file with SPI capture and generate packets
# Return a list with the compiled packets
# 
# Packet format:
# [time, packetString, repeats]
# 
def getPacketsFromFile(filename, identifier, analyser):
    packets = []
    with open(filename, 'rb') as csvfile:
        reader = csv.reader(csvfile)

        # Ignore first line
        reader.next() 
        
        packet_index = 0
        packet_time = 0

        # print starttime
        current_packet = array.array('B')
        old_packet = []
        dupe_count = 0
        
        current_index = -1

        # For each line
        for row in reader:
            
            if analyser == "saleae" :
                # Sometimes row[1] is just ''
                try:
                    current_index = int(row[1])
                except:
                    current_index = -1
            if analyser == "openbench" :
                if row[4] == "CS_LOW" :
                    current_index += 1
                    continue
                if row[4] == "CS_HIGH" :
                    continue
            
            if current_index != packet_index:
                if old_packet == current_packet:
                    dupe_count += 1
                else:
                    packetString = ""
                    for byte in old_packet:
                        packetString += format(byte, '02x') + ' '

                    packetString = packetString.strip()

                    # Ignore empty packets
                    if(len(packetString) > 0):
                        packetItem = [packet_time, packetString, dupe_count]
                        if(identifier):
                            packetItem.append(identifier)

                        packets.append(packetItem)

                    dupe_count = 0

                packet_index = current_index
                old_packet = current_packet
                current_packet = array.array('B')
                
                if analyser == "saleae" :
                    packet_time = float(row[0])
                if analyser == "openbench" :
                    multiplier = 1
                    if "?s" in row[1]:
                        multiplier = 0.001
                    packet_time = float(row[1].strip(" ?sm").replace(",",".")) * multiplier

            if analyser == "saleae" :
                current_packet.append(int(row[2].strip("MOSI: "), 16)) 
            if analyser == "openbench" :
                current_packet.append(int(row[5])) 
                       

    return packets

# Get a list of packets in the above format and print them
def printPackets(packets, decode, pattern):
    if len(packets) == 0 :
        sys.stdout.write("No packet found !")

    for packet in packets:
        
        
        if pattern != "" :
            toWrite = a7105.decodeSPIPacket(packet[1])
            
            if pattern in toWrite :
            
                if(len(packet) > 3):
                    sys.stdout.write(packet[3] + ' ')        
            
                sys.stdout.write(format(packet[0], 'f') + ' ')

                if decode == False:
                    sys.stdout.write(packet[1]);
                else:
                    sys.stdout.write(toWrite)
                if packet[2] > 0:
                    sys.stdout.write(' (Repeated ' + str(packet[2]) + ' times)')
                
                sys.stdout.write('\n')
        else : 
        
            if(len(packet) > 3):
                sys.stdout.write(packet[3] + ' ')        
        
            sys.stdout.write(format(packet[0], 'f') + ' ')

            if decode == False:
                sys.stdout.write(packet[1]);
            else:
                toWrite = a7105.decodeSPIPacket(packet[1])
                if fifo == True : 
                    if "R FIFO" in toWrite :
                        sys.stdout.write(toWrite)
                else :
                    sys.stdout.write(toWrite)

            if packet[2] > 0:
                sys.stdout.write(' (Repeated ' + str(packet[2]) + ' times)')
            
            sys.stdout.write('\n')

def getTime(packet):
    return packet[0]

# 
# Main Code
# 

parser = argparse.ArgumentParser()
parser.add_argument('-r','--remote', type=str)
parser.add_argument('-q','--quad', type=str)
parser.add_argument('-d', action="store_true", default=False) 
# Can be saleae or openbench
parser.add_argument('-a', '--analyser', type=str, default="saleae")
# Search a pattern
parser.add_argument('-p', '--pattern', type=str, default="")

ARGS = parser.parse_args()

packets = []

if ARGS.remote:
    packets.extend(getPacketsFromFile(ARGS.remote, 'remote', ARGS.analyser))

if ARGS.quad:
    packets.extend(getPacketsFromFile(ARGS.quad, 'quad  ', ARGS.analyser))

# Combine both packets and sort by timestamp for a nice intermixed printout
printPackets(sorted(packets, key=getTime), ARGS.d, ARGS.pattern)
