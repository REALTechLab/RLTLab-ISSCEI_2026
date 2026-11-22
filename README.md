# RLTLab-ISSCEI_2026

## 📖 Introduction

Repository này lưu trữ mã nguồn, tài liệu kỹ thuật và mô hình triển khai của đội thi đến từ **REALTech Laboratory** (*Viện Công nghệ Hàng không Vũ trụ - Đại học Duy Tân*), tham gia cuộc thi **Smart Campus 2026** trong khuôn khổ Hội thảo Khoa học Quốc tế **ISSCEI 2026** (*International Symposium: Smart City - Experiences and Innovations*) do Viện Công nghệ Quốc tế DNIIT (ĐH Đà Nẵng) và Sở Khoa học & Công nghệ TP. Đà Nẵng phối hợp tổ chức.

* **Chủ đề cuộc thi:** *"Công nghệ thế hệ mới cho liên kết xanh Nước - Năng lượng - Thực phẩm - Hệ sinh thái (WEFE) bền vững và Đô thị thích ứng"*.
* **Lĩnh vực trọng tâm:** `[Chọn lĩnh vực của bạn: Thiết bị không người lái (UAV) trong đô thị thông minh / Giám sát thông minh (Smart Monitoring) / Nông nghiệp thông minh & Kinh tế tuần hoàn / Năng lượng xanh & Giao thông thông minh]`
* **Mục tiêu dự án:** `[Mô tả ngắn gọn 2-3 câu về bài toán thực tiễn mà dự án giải quyết tại khuôn viên đại học thông minh hoặc đô thị Đà Nẵng, ví dụ: Ứng dụng UAV tự hành kết hợp thị giác máy tính (AI/Edge Computing) để giám sát môi trường, tối ưu tài nguyên nước và phát hiện sớm rủi ro hạ tầng đô thị theo thời gian thực.]`

---

## ✨ Cấu trúc thư mục

```text
RLTLab-ISSCEI_2026/
├── docs/                   # Tài liệu thuyết minh, poster, slide báo cáo ISSCEI 2026
├── hardware/               # Sơ đồ mạch, thiết kế 3D, thông số phần cứng (nếu có)
├── src/                    # Mã nguồn chính của hệ thống
│   ├── ai_models/          # Mô hình học sâu, xử lý ảnh & trọng số
│   ├── control/            # Thuật toán điều khiển nhúng / ROS / Firmware
│   └── dashboard/          # Giao diện giám sát Web/App
├── scripts/                # Các script hỗ trợ cài đặt, chạy benchmark & test
├── requirements.txt        # Danh sách thư viện Python phụ thuộc
├── LICENSE                 # Giấy phép mã nguồn mở
└── README.md               # Tài liệu hướng dẫn tổng quan
```

---

## ⚙️ Requirements

### 1. Phần cứng (Hardware)
* **Thiết bị xử lý trung tâm / Biên:** `[VD: NVIDIA Jetson Orin Nano / Raspberry Pi 5 / STM32]`
* **Nền tảng di động / Cảm biến:** `[VD: Khung UAV Quadcopter / Cảm biến chất lượng nước, nhiệt độ, độ ẩm, Camera RGB/Thermal]`
* **Trạm mặt đất (GCS):** Laptop/PC hỗ trợ Wi-Fi/Telemetry.

### 2. Phần mềm (Software)
* **Hệ điều hành:** Ubuntu 22.04 LTS / Windows 11
* **Ngôn ngữ & Framework:** Python >= 3.10, C++17, `[ROS 2 Humble / PyTorch / OpenCV / FastAPI / ReactJS]`
* **Công cụ khác:** Git, Docker (khuyên dùng)

---

## 🚀 Installation & Usage

### Bước 1: Clone repository

```bash
git clone https://github.com/REALTechLab/RLTLab-ISSCEI_2026.git
cd RLTLab-ISSCEI_2026
```

### Bước 2: Thiết lập môi trường

```bash
# Tạo môi trường ảo Python
python3 -m venv venv
source venv/bin/activate  # Trên Windows dùng: venv\Scripts\activate

# Cài đặt các thư viện cần thiết
pip install --upgrade pip
pip install -r requirements.txt
```

### Bước 3: Cấu hình tham số

Sao chép tệp cấu hình mẫu và điều chỉnh các thông số kết nối (API keys, cổng COM, broker MQTT, tọa độ bản đồ):

```bash
cp .env.example .env
```

### Bước 4: Khởi chạy hệ thống

```bash
# Chạy module xử lý chính / AI Inference
python src/main.py --config configs/default.yaml

# (Tùy chọn) Khởi chạy giao diện Dashboard
cd src/dashboard && npm install && npm run dev
```

---

## 📊 Demo & Results

* **Video Demo sản phẩm:** `[Chèn link YouTube / Google Drive demo tại đây]`
* **Kết quả thử nghiệm thực tế:**
  * Độ chính xác mô hình nhận diện: `XX.X%` (mAP@0.5)
  * Thời gian phản hồi hệ thống (End-to-end latency): `< XX ms`
  * Hiệu quả tiết kiệm năng lượng / tài nguyên: `[Điền số liệu thực nghiệm]`

---

## 👥 Team & Acknowledgments

Dự án được nghiên cứu và phát triển bởi nhóm sinh viên & nghiên cứu viên thuộc **REALTech Laboratory** (*Research, Engineering, Applied, and Learning Laboratory*):

* **Giảng viên hướng dẫn:** TS. Trần Lê Thăng Đồng (*Giám đốc REALTech Lab - Viện Công nghệ Hàng không Vũ trụ, ĐH Duy Tân*)
* **Thành viên nhóm thực hiện:**
  1. `[Họ và Tên Thành viên 1]` — Trưởng nhóm / Hệ thống nhúng & Điều khiển
  2. `[Họ và Tên Thành viên 2]` — AI & Thị giác máy tính
  3. `[Họ và Tên Thành viên 3]` — Phát triển Phần mềm & Dashboard
  4. `[Họ và Tên Thành viên 4]` — Thiết kế Phần cứng & Thử nghiệm thực địa

**Lời cảm ơn:** Nhóm xin chân thành cảm ơn Ban Tổ chức Hội thảo **ISSCEI 2026**, **Viện Công nghệ Quốc tế DNIIT (ĐH Đà Nẵng)**, **Sở Khoa học & Công nghệ TP. Đà Nẵng** cùng **REALTech Laboratory - Viện IAE - Đại học Duy Tân** đã tạo điều kiện về không gian nghiên cứu, trang thiết bị và cơ hội kết nối để chúng tôi hiện thực hóa giải pháp này.

---

## 📄 License

Dự án này được phân phối dưới giấy phép **MIT License** — xem chi tiết tại tệp [LICENSE](LICENSE) để biết thêm thông tin.

---

<div align="center">
  <sub>Phát triển với ❤️ tại <b>REALTech Laboratory</b> — Đà Nẵng, Việt Nam (2026)</sub>
</div>
```