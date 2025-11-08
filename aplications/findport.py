import serial.tools.list_ports as serial_port

def find_arduino_port():
    """
    시스템에 연결된 Arduino 보드의 포트를 찾아서 반환합니다.
    여러 Arduino가 연결된 경우 "Arduino" 또는 "Mega"가 포함된 첫 번째 포트를 반환합니다.
    
    Returns:
        str: Arduino가 연결된 포트 이름 (예: 'COM3')
             Arduino를 찾지 못한 경우 None 반환
    """
    port_lists = serial_port.comports()
    
    # 사용 가능한 포트 출력
    for port in port_lists:
        print(f"{port.device} - {port.description}")
    
    # Arduino 포트 찾기
    for port in port_lists:
        if "Arduino" in port.description or "Mega" in port.description:
            return port.device
            
    return None

if __name__ == "__main__":
    # 테스트용 코드
    port = find_arduino_port()
    if port:
        print(f"\nArduino found at: {port}")
    else:
        print("\nNo Arduino found")
