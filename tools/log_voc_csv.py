"""ESP32 USB CSV 저장기. 실행: py log_voc_csv.py --port COM7
--start --origin-confirmed: 수동 정렬 완료 후 연결2초 뒤 z/a 전송(설정 검증된 정지 프린터 실험에서만).
시리얼 모니터를 먼저 닫으세요. Ctrl+C는 기록 종료이며 기구 정지는 아닙니다.
기구 정지는 별도 터미널에서 x 전송 또는 --stop-on-exit 사용.
"""
import argparse
import csv
from datetime import datetime, timezone
from pathlib import Path
import time

FIELDS = 'seq,uptime_ms,voc_in_a,voc_in_b,voc_out_a,voc_out_b,valid_mask,control_voc,fan_command_pct,state,replacements,cooldown_s,fault'.split(',')

def parse_row(line):
    """진단문/헤더/잘린 행은 데이터로 저장하지 않는다."""
    row = next(csv.reader([line]))
    if len(row) != len(FIELDS):
        return None
    try:
        seq, ms = int(row[0]), int(row[1])
        mask, fan = int(row[6]), int(row[8])
        if seq < 1 or ms < 0 or not 0 <= mask <= 15 or not 0 <= fan <= 100:
            return None
        for i in range(4):
            if mask & (1 << i):
                if not 1 <= int(row[i+2]) <= 500:
                    return None
            elif row[i+2] != '':
                return None
        int(row[10]); int(row[11])
    except ValueError:
        return None
    return row

def main():
    import serial  # py -m pip install pyserial
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', required=True)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--start', action='store_true')
    parser.add_argument('--origin-confirmed', action='store_true', help='수동 시작위치 정렬 완료: z를 보내 현재 위치를0으로 설정')
    parser.add_argument('--stop-on-exit', action='store_true')
    args = parser.parse_args()
    if args.start and not args.origin_confirmed:
        parser.error('--start requires --origin-confirmed after manual shuttle alignment')
    path = args.output or Path('voc_' + datetime.now().strftime('%Y%m%d_%H%M%S') + '.csv')
    # x 모드: 기존 실험 파일을 실수로 덮어쓰지 않는다.
    with path.open('x', newline='', encoding='utf-8-sig') as file:
        writer = csv.writer(file)
        writer.writerow(['pc_time_utc', 'boot_segment'] + FIELDS)
        file.flush()
        with serial.Serial(args.port, 115200, timeout=0.5) as port:
            print('저장:', path.resolve(), '| Ctrl+C: 기록 종료')
            # 일부 보드는 포트 연결 시 리셋될 수 있다. 시작 명령을 약간 늦춘다.
            start_at = time.monotonic() + 2 if args.start else None
            segment, previous, pending = 0, None, b''
            try:
                while True:
                    if start_at is not None and time.monotonic() >= start_at:
                        port.write(b'z\na\n'); start_at = None
                    pending += port.read(port.in_waiting or 1)
                    while b'\n' in pending:
                        raw, pending = pending.split(b'\n', 1)
                        row = parse_row(raw.decode('utf-8', errors='replace').strip())
                        if row is None:
                            continue
                        now_pair = int(row[0]), int(row[1])
                        if previous is not None and (now_pair[0] <= previous[0] or now_pair[1] < previous[1]):
                            segment += 1  # 보드 리셋 또는 millis wrap 구간을 구분
                        previous = now_pair
                        writer.writerow([datetime.now(timezone.utc).isoformat(), segment] + row)
                        file.flush()  # 매 행 저장; 프로세스 종료 시 기록 손실 최소화
                        print('VOC:', '/'.join(row[2:6]), '팬:', row[8], '상태:', row[9], '오류:', row[12])
                    if len(pending) > 4096:
                        pending = b''  # 비정상 무개행 입력의 무한 메모리 증가 방지
            except KeyboardInterrupt:
                print('\n기록 종료')
            finally:
                if args.stop_on_exit:
                    port.write(b'x\n'); port.flush()

if __name__ == '__main__':
    main()
