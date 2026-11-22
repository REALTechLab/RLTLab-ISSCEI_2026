# RLTLab-ISSCEI_2026

## 📖 Introduction

Repository này lưu trữ mã nguồn, tài liệu kỹ thuật và mô hình triển khai của đội thi đến từ **REALTech Laboratory** (*Viện Công nghệ Hàng không Vũ trụ - Đại học Duy Tân*), tham gia cuộc thi **Smart Campus 2026** trong khuôn khổ Hội thảo Khoa học Quốc tế **ISSCEI 2026** (*International Symposium: Smart City - Experiences and Innovations*) do Viện Công nghệ Quốc tế DNIIT (ĐH Đà Nẵng) và Sở Khoa học & Công nghệ TP. Đà Nẵng phối hợp tổ chức.

* **Chủ đề cuộc thi:** *"Công nghệ thế hệ mới cho liên kết xanh Nước - Năng lượng - Thực phẩm - Hệ sinh thái (WEFE) bền vững và Đô thị thích ứng"*.
* **Lĩnh vực trọng tâm:** `Thiết bị không người lái (UAV) trong đô thị thông minh / Giám sát thông minh (Smart Monitoring) / Nông nghiệp thông minh & Kinh tế tuần hoàn / Năng lượng xanh & Giao thông thông minh`
* **Mục tiêu dự án:** `Xây dựng thế giới 3D tích hợp bản đồ địa lý trong môi trường mô phỏng Gazebo, ứng dụng để nghiên cứu các bài toán điều khiển thiết bị bay không người lái (UAV) và hỗ trợ tạo quỹ đạo bay trong thế giới thực từ dữ liệu các điểm định vị cục bộ trong thế giới ảo. `

---

## ✨ Cấu trúc thư mục

```text
RLTLab-ISSCEI_2026/
├── documents/              # Tài liệu thuyết minh, poster, slide báo cáo ISSCEI 2026
├── models/			# Mô hình drone x500
├── worlds/                 # Thế giới 3D
├── scripts/                # Các script hỗ trợ cài đặt
├── requirements.txt        # Danh sách thư viện Python phụ thuộc
├── LICENSE                 # Giấy phép mã nguồn mở
└── README.md               # Tài liệu hướng dẫn tổng quan
```

---

## ⚙️ Requirements

### 1. Phần cứng (Hardware)
* **Thiết bị xử lý trung tâm / Biên:** `Mini PC Intel Core i7, 32GB Ram, 1TB SSD`
* **Nền tảng di động / Cảm biến:** `Quadrotor X650`
* **Trạm mặt đất (GCS):** `Laptop/PC hỗ trợ Wi-Fi/Telemetry.`

### 2. Phần mềm (Software)
* **Hệ điều hành:** Ubuntu 22.04 LTS / Windows 11
* **Trình mô phỏng:** Gazebo Sim
* **Ngôn ngữ & Framework:** Python >= 3.10, C++17, `[ROS 2 Humble / PyTorch / OpenCV  / ReactJS]`
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
### Bước 3: Khởi chạy hệ thống

```bash
# Chạy module xử lý chính / AI Inference
python src/main.py --config configs/default.yaml

# (Tùy chọn) Khởi chạy giao diện Dashboard
cd src/dashboard && npm install && npm run dev
```


## 👥 Team & Acknowledgments

Dự án được nghiên cứu và phát triển bởi nhóm sinh viên & nghiên cứu viên thuộc **REALTech Laboratory** (*Research, Engineering, Applied, and Learning Laboratory*):

* **Giảng viên hướng dẫn:** TS. Trần Lê Thăng Đồng (*Trưởng nhóm REALTech Lab - Viện Công nghệ Kỹ thuật Hàng không Vũ trụ, Đại học Duy Tân*)
* **Thành viên nhóm thực hiện:**
  1. `Lê Hoàng Minh Huy` — Trưởng nhóm
  2. `Phan Nguyễn Phước Quang ` — Thành viên
  3. `Võ Hồng Quân` — Thành viên

**Lời cảm ơn:** Nhóm xin chân thành cảm ơn Ban Tổ chức Hội thảo **ISSCEI 2026**, **Viện Công nghệ Quốc tế DNIIT (ĐH Đà Nẵng)**, **Sở Khoa học & Công nghệ TP. Đà Nẵng** cùng **REALTech Laboratory - Viện IAE - Đại học Duy Tân** đã tạo điều kiện về không gian nghiên cứu, trang thiết bị và cơ hội kết nối để chúng tôi hiện thực hóa giải pháp này.

---

## 📄 License

Dự án này được phân phối dưới giấy phép **MIT License** — xem chi tiết tại tệp [LICENSE](LICENSE) để biết thêm thông tin.

---

<div align="center">
  <sub>Phát triển với ❤️ tại <b>REALTech Laboratory</b> — Đà Nẵng, Việt Nam (2026)</sub>
</div>