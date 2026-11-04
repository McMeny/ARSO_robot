"""Makes the HTTPS certificate the Quest needs (it only allows VR on secure pages).

  ros2 run walker_vr make_certs            certificate for localhost + this PC's IP addresses
  ros2 run walker_vr make_certs 192.168.1.42   ...plus extra IP addresses

Writes ~/.walker_vr/cert.pem and key.pem. Self-signed, so the Quest browser shows a
warning the first time - choose Advanced -> Proceed (README step A7).
"""
import os
import socket
import subprocess
import sys

CERT_DIR = os.path.expanduser('~/.walker_vr')


def local_ips():
    ips = {'127.0.0.1'}
    try:
        out = subprocess.run(['hostname', '-I'], capture_output=True, text=True).stdout
        ips.update(ip for ip in out.split() if ':' not in ip)
    except OSError:
        pass
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(('8.8.8.8', 80))
        ips.add(s.getsockname()[0])
        s.close()
    except OSError:
        pass
    return sorted(ips)


def main():
    os.makedirs(CERT_DIR, exist_ok=True)
    ips = sorted(set(local_ips() + [a for a in sys.argv[1:] if a.count('.') == 3]))
    san = ','.join(['DNS:localhost'] + [f'IP:{ip}' for ip in ips])
    cert, key = os.path.join(CERT_DIR, 'cert.pem'), os.path.join(CERT_DIR, 'key.pem')
    cmd = ['openssl', 'req', '-x509', '-newkey', 'rsa:2048', '-nodes', '-days', '825',
           '-subj', '/CN=walker-vr', '-addext', f'subjectAltName={san}', '-keyout', key, '-out', cert]
    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode != 0:
        print('FAILED to make the certificate:\n' + result.stderr)
        sys.exit(1)
    print(f'Certificate written to {cert}')
    print('Valid for: localhost, ' + ', '.join(ips))
    print('If the Quest uses a different address for this PC, run again with it, e.g.:'
          '  ros2 run walker_vr make_certs 192.168.1.42')


if __name__ == '__main__':
    main()
