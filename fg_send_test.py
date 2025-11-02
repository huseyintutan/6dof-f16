# fg_send_test.py
import socket, time, math

HOST, PORT = "127.0.0.1", 5500
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Başlangıç konumu (örnek: İstanbul)
lat0 = 41.015137
lon0 = 28.979530
alt  = 1200.0  # metre

t0 = time.time()
rate = 30.0
dt = 1.0 / rate

print("Sending to FlightGear on udp://127.0.0.1:5500 ... Ctrl+C to stop.")
try:
    while True:
        t = time.time() - t0
        # küçük bir daire hareketi (~50 m yarıçap) ve hafif bank
        dlat = (50.0/111_320.0) * math.cos(0.2*t)   # ~1 deg = 111.32 km
        dlon = (50.0/(111_320.0*math.cos(math.radians(lat0)))) * math.sin(0.2*t)

        lat = lat0 + dlat
        lon = lon0 + dlon
        roll_deg  = 10.0*math.sin(0.4*t)
        pitch_deg =  2.0*math.sin(0.3*t)
        heading   = ( (math.degrees(0.2*t) % 360.0) )

        line = f"{lat:.8f},{lon:.8f},{alt:.2f},{roll_deg:.3f},{pitch_deg:.3f},{heading:.3f}\n"
        sock.sendto(line.encode("ascii"), (HOST, PORT))
        time.sleep(dt)
except KeyboardInterrupt:
    pass
