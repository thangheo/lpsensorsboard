# low power sensorsboard
một mạng cảm biến tự làm chạy bằng pin với nhiều loại cảm biến cho nhà thông minh.
PCB và sơ đồ được thiết kế bằng Kicad 6.0.
một phần của cấu hình firmare được tạo bởi STM CubeMX.

Ở phiên bản hiện tại, các cảm biến được hỗ trợ là Cường độ ánh sáng sử dụng điện trở quang (ánh sáng rất yếu, <5 Lux),
Cảm biến CO2/độ ẩm/nhiệt độ từ sensirion SCD41(từ Seeedstudio
với đầu nối Grove), cảm biến âm thanh (Micrô), cảm biến CO/NH3, cảm biến AM2320 cho độ ẩm và nhiệt độ.
Tôi chưa thử nghiệm trên phần cứng thực vì tôi chỉ thiết kế PCB và lập trình Firmware vài tuần trước.
Vi điều khiển là STM32L051 với lõi Cortex M0+, ở chế độ STANDBY, mức tiêu thụ điện năng có thể giảm xuống mức thấp nhất là vài micro amp.
Theo tính toán của tôi, thiết bị có thể chạy tới 7-8 tháng với thời gian đo 2 phút với
 Pin Lithium 18658 (1000mAh). Nếu không có cảm biến CO/NH3, nó có thể chạy tới nhiều năm mà không cần sạc lại
  pin vì cảm biến này tiêu thụ một nửa năng lượng (35mA trong tổng số 70mA).
![alt text](https://github.com/thangheo/lpsensorsboard/blob/main/images/pcb1.png?raw=true)
![alt text](https://github.com/thangheo/lpsensorsboard/blob/main/images/pcb2.png?raw=true)

1. Cấu trúc dữ liệu gửi đi (bổ sung sau)

typedef struct {
  unsigned int temperature;
  unsigned int humidity;
  unsigned int AM2320_temperature;
  unsigned int AM2320_humidity;
  unsigned int audio;
  unsigned int light;
  unsigned int CO2;
  unsigned int NH3;
} 