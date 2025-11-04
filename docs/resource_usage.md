# Sistem Kaynak Kullanım Analizi

## Mevcut Ayarlar (Optimize Edilmiş)

### Simülasyon Parametreleri
- **Zaman Adımı (DT)**: 0.01 s (100 Hz iç simülasyon hızı)
- **Toplam Süre**: 60 s
- **Toplam Adım**: 6,000 adım
- **Real-time Senkronizasyon**: AÇIK (REAL_TIME = True)

### Güncelleme Hızları (Optimize Edilmiş)
- **Dashboard**: 2 Hz (her 0.5 s güncelleme)
- **FlightGear UDP**: 10 Hz (her 0.1 s gönderim)
- **Sensör Hesaplama**: 2 Hz (her 0.5 s)
- **Sensör Log**: 1 Hz (her 1 s konsola yazdırma)

### Bellek (RAM) Kullanımı

#### Dashboard Veri Yapıları
- **Pencere Süresi**: 20 s
- **Nokta Sayısı**: 20 s / 0.01 s = **2,000 nokta**
- **Veri Yapısı**: `deque` (döngüsel buffer, sabit boyut)

**Combined Dashboard (State + Sensors):**
- State verileri: ~10 deque × 2,000 float × 8 byte = **~160 KB**
- Sensör verileri: ~10 deque × 2,000 float × 8 byte = **~160 KB**
- **Toplam Dashboard**: **~320 KB**

**Ayrı Sensor Dashboard (opsiyonel):**
- ~15 deque × 2,000 float × 8 byte = **~240 KB**

**Toplam Bellek (Dashboard):** ~320-560 KB

#### Simülasyon Verileri
- State dictionary: ~10 float × 8 byte = **80 byte**
- Quaternion: 4 float × 8 byte = **32 byte**
- NED position: 3 float × 8 byte = **24 byte**
- **Toplam Simülasyon State**: **~136 byte**

#### UDP Buffer
- Socket buffer: **64 KB** (FlightGear için)

### CPU Kullanımı

#### Her Simülasyon Adımında (100 Hz):
1. **RK4 Dinamik Hesaplama**: 4 kez `f_dot()` çağrısı
   - Aerodinamik kuvvet/moment hesaplama
   - Gravite hesaplama
   - Dönüşüm matrisleri
   - **~CPU: Orta**

2. **Quaternion Entegrasyonu**: RK4 ile 4 aşama
   - **~CPU: Düşük**

3. **Pozisyon Entegrasyonu**: RK4 ile NED koordinatları
   - **~CPU: Düşük**

#### Periyodik İşlemler:

**Her 5. Adım (20 Hz) - FlightGear:**
- Geodetic dönüşüm (lat/lon)
- UDP gönderim
- **~CPU: Çok Düşük**

**Her 50. Adım (2 Hz) - Dashboard:**
- Veri ekleme (deque.append)
- Grafik güncelleme (matplotlib)
- **~CPU: Yüksek** (matplotlib render)

**Her 50. Adım (2 Hz) - Sensörler:**
- IMU hesaplama
- GPS hesaplama
- Air data hesaplama
- **~CPU: Orta**

### Ağ Kullanımı (Network)

**FlightGear UDP:**
- Gönderim Hızı: 10 Hz
- Paket Boyutu: ~50 byte (lat,lon,alt,roll,pitch,heading)
- Bant Genişliği: 10 Hz × 50 byte = **500 byte/s = ~4 Kbps**

### Disk Kullanımı
- Python kodları: ~50 KB
- Veri dosyaları (JSON): ~5 KB
- **Toplam**: **~55 KB**

## Toplam Kaynak Kullanım Özeti

### RAM (Bellek)
- Dashboard: ~320-560 KB
- Simülasyon state: ~136 byte
- UDP buffer: 64 KB
- Python runtime: ~10-50 MB (Python kendisi)
- **Toplam Tahmini**: **~15-60 MB** (Python runtime dahil)

### CPU
- **Normal Durum**: %10-30 (düşük-orta)
- **Dashboard Render**: %40-60 (yüksek, 2 Hz'de)
- **Ortalama**: %15-25

### Ağ
- **UDP Gönderim**: ~4 Kbps (çok düşük)

## Optimizasyon Önerileri

### Daha Az Kaynak İçin:
1. **Dashboard'u Kapat**: `ENABLE_DASHBOARD = False`
   - RAM: ~320 KB tasarruf
   - CPU: %30-40 azalma

2. **FlightGear Gönderimini Azalt**: `FG_UPDATE_RATE = 5.0`
   - CPU: %5 azalma
   - Ağ: %50 azalma

3. **Sensör Hesaplamayı Azalt**: Dashboard ile aynı hızda
   - CPU: %10 azalma

4. **Dashboard Pencere Boyutunu Küçült**: `PLOT_WINDOW_SEC = 10.0`
   - RAM: %50 azalma (~160 KB tasarruf)

### Performans İçin:
1. **Real-time Senkronizasyonu Kapat**: `REAL_TIME = False`
   - CPU: %20-30 azalma (ama FlightGear ile senkron kaybolur)

2. **Dashboard Güncelleme Hızını Azalt**: `DASH_RATE = 1.0`
   - CPU: %40 azalma (render sıklığı)

## Mevcut Durum: Optimize Edilmiş ✅

Şu anki ayarlar performans ve görselleştirme arasında iyi bir denge sağlıyor:
- ✅ Düşük bellek kullanımı (~500 KB dashboard)
- ✅ Orta CPU kullanımı (%15-25)
- ✅ Çok düşük ağ kullanımı (~4 Kbps)
- ✅ Gerçek zamanlı senkronizasyon aktif
- ✅ Tüm veriler görselleştiriliyor

