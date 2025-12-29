# ECG Atrial Fibrillation Detection Using CNN

This repository contains a Python script for detecting **Atrial Fibrillation (AF)** from ECG signals.  
The script integrates **signal preprocessing, RR interval extraction, and a CNN / CNN-LSTM model** for classification.

---

## 📌 Project Overview
- **Input:** ECG signal (3-lead configuration)  
- **Dataset:** MIT-BIH Atrial Fibrillation Database (AFDB)  
- **Method:** Convolutional Neural Network (CNN) / CNN-LSTM  
- **Output:** AF / Normal classification  

---

## 🧠 Workflow
1. **Signal Preprocessing**
   - Bandpass filtering  
   - Signal segmentation  
   - Noise removal  

2. **Feature Extraction**
   - RR interval calculation  
   - Normalization  

3. **Model**
   - Conv1D layers  
   - MaxPooling1D  
   - LSTM  
   - Dropout  
   - Dense output (Softmax)  

4. **Evaluation**
   - K-Fold Cross Validation  
   - Accuracy, Precision, Recall, F1-Score  
   - Confusion Matrix  

---

## 📊 Model Performance (K-Fold Cross Validation)

- **Mean Accuracy:** 0.9948 ± 0.0049  
- **Average ROC AUC:** 0.988  
- **Average Precision-Recall AUC:** 0.991  
- **Average Precision:** 0.978  
- **Average Recall:** 0.978  
- **Average F1-Score:** 0.978  

**Average Confusion Matrix:**

|               | Pred Normal | Pred AF |
|---------------|------------|---------|
| True Normal    | 231        | 2       |
| True AF        | 8          | 218     |

✅ **Interpretation:**  
The model demonstrates **very high performance**, with >99% accuracy and F1-score of 0.978, indicating reliable detection of atrial fibrillation while minimizing misclassifications.

---

## ⚙️ Requirements
```bash
pip install -r requirements.txt
