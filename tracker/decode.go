package main

import (
	"bufio"
	"bytes"
	"encoding/hex"
	"flag"
	"fmt"
	"io"
	"os"
)

func main() {
	gps := flag.Bool("gps", false, "decode gps packets")
	imu := flag.Bool("imu", false, "decode IMU packets")
	flag.Parse()

	if *imu == *gps {
		fmt.Fprintln(os.Stderr, "One of -gps or -imu required")
		os.Exit(1)
	}

	if *imu {
		fmt.Println("dev,seq,millis,calib,qua-w,qua-x,qua-y,qua-z,acc-x,acc-y,acc-z")
	}
	if *gps {
		fmt.Println("dev,time,valid,lat,lon,kts,heading")
	}

	in := bufio.NewReader(os.Stdin)
	n := 1
	for {
		// Read line.
		line, err := in.ReadBytes('\n')
		if err == io.EOF {
			break
		}
		if err != nil {
			fmt.Fprintln(os.Stderr, err.Error())
			os.Exit(1)
		}
		if !bytes.HasPrefix(line, []byte("PKT>")) {
			continue
		}

		// Convert hex to bytes.
		line = line[4 : len(line)-1] // remove lieading PKT> and trailing \n
		if len(line) < 5 {
			continue
		}
		pkt := make([]byte, (len(line)-3)/2)
		_, err = hex.Decode(pkt, line[:len(pkt)*2])
		if err != nil {
			fmt.Fprintln(os.Stderr, err.Error())
			os.Exit(1)
		}

		// Verify checksum.
		var chk byte
		for _, c := range pkt {
			chk += c
		}
		got := [1]byte{}
		hex.Decode(got[:], line[len(line)-2:])
		if got[0] != chk {
			fmt.Fprintf(os.Stderr, "CHK ERR, calc %02x, got %x\n", chk, line)
			continue
		}

		switch {
		case pkt[0] == 0x8A && *gps:
			gpsStr := gpsNavToCSV(pkt[1:])
			fmt.Printf("GPS,%s\n", gpsStr)
		case pkt[0] == 0x8D && *imu:
			if len(pkt) != 5+7*2 { // 3 byte padding
				fmt.Fprintf(os.Stderr,
					"Incorrect IMU paket length, got %d, expect %d\n",
					len(pkt), 5+7*2)
				continue
			}
			millis := uint32(pkt[1]) | (uint32(pkt[2]) << 8) | (uint32(pkt[3]) << 16)
			calib := pkt[4+14]
			fmt.Printf("IMU,%d,%.3f,%#02x,", n, float64(millis)/1000, calib)
			n++
			var data [7]float64
			for i := range data {
				data[i] = float64(int16(pkt[4+2*i]) + (int16(pkt[4+2*i+1]) << 8))
			}
			// Output quaternions.
			fmt.Printf("%.5f,%.5f,%.5f,%.5f,", data[0]/16384, data[1]/16384,
				data[2]/16384, data[3]/16384)
			// Output acceleration.
			fmt.Printf("%.5f,%.5f,%.5f\n", data[4], data[5], data[6])
		}
	}

	fmt.Fprintf(os.Stderr, "Decoded %d records\n", n)
}
