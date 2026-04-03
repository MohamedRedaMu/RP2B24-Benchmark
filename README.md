# RP2B24-Benchmark
Reda Path Planning Benchmark Library 2024 (RP2B24)

Official MATLAB implementation of the RP2B24 benchmark library for path-planning research.

## 📄 Related Paper

Mohamed Reda, Ahmed Onsy, Amira Y. Haikal, and Ali Ghanbari  
**A novel reinforcement learning-based multi-operator differential evolution with cubic spline for the path planning problem**  
Artificial Intelligence Review, Volume 58, Article 142, 2025  
DOI: https://doi.org/10.1007/s10462-025-11129-6

---

## Overview

RP2B24 is a standardized benchmark library for evaluating path-planning algorithms under diverse obstacle configurations and environmental layouts.

The benchmark library provides:

- 50 path-planning scenarios
- multiple environment categories
- support for optimization-based and learning-based path planners
- standardized model generation for fair comparison

According to the published paper, RP2B24 contains benchmark problems grouped into five main categories:
- open field models
- single obstacle models
- multiple small obstacle models
- narrow passage models
- maze-like models. 

---

## Main Purpose

This repository provides the benchmark generator used in the QSMODE/DQSMODE path-planning framework and related studies.

It is intended for:
- testing path-planning algorithms
- comparing optimization methods on identical scenarios
- generating reproducible benchmark environments for research papers

---

## 📂 Repository Structure

```text
RP2B24-Benchmark/
│
├── CreateModel_RP2B24_Benchmark.m
├── README.md
├── LICENSE
├── CITATION.cff
├── .gitignore
└── .gitattributes
```

---

## ⚙️ Requirements

- MATLAB (recommended R2023a or later)

No external toolbox is strictly required for the benchmark definition file itself, unless your downstream path-planning algorithms use additional MATLAB toolboxes.

---

## ▶️ How to Use

Open MATLAB, set the current folder to the repository root, then call the benchmark generator:

```matlab
model = CreateModel_RP2B24_Benchmark(modelType, modelNum, boundaries, fNo);
```

### Example

```matlab
boundaries.xmin = -10;
boundaries.xmax = 10;
boundaries.ymin = -10;
boundaries.ymax = 10;

modelType = 4;
modelNum  = 8;
fNo       = 1;

model = CreateModel_RP2B24_Benchmark(modelType, modelNum, boundaries, fNo);
```

---

## 🔧 Input Parameters

The function format is:

```matlab
CreateModel_RP2B24_Benchmark(modelType, modelNum, boundaries, fNo)
```

### Parameters Description

- `modelType` : benchmark category  
  Supported values:
  - `1` = Open Field Model
  - `2` = Single Obstacle Model
  - `3` = Multiple Small Obstacles Model
  - `4` = Narrow Passage Model
  - `5` = Maze-like Model

- `modelNum` : configuration number within the selected category  
  Valid range depends on the chosen `modelType`

- `boundaries` : structure containing
  - `xmin`
  - `xmax`
  - `ymin`
  - `ymax`

- `fNo` : benchmark identifier used for naming / indexing purposes

---

## 📌 Notes

- The benchmark definitions are intended for research and reproducible evaluation
- Obstacle shapes are represented using circular and rectangular abstractions
- The benchmark output includes:
  - start point
  - target point
  - obstacle positions
  - obstacle sizes
  - search-space boundaries
  - benchmark metadata

The related paper states that RP2B24 includes 50 benchmark problems organized into the five categories above for evaluating path-planning methods. 

---

## 📚 Citation

If you use this benchmark library, please cite:

```bibtex
@article{reda2025novel,
  title   = {A novel reinforcement learning-based multi-operator differential evolution with cubic spline for the path planning problem},
  author  = {Reda, Mohamed and Onsy, Ahmed and Haikal, Amira Y. and Ghanbari, Ali},
  journal = {Artificial Intelligence Review},
  volume  = {58},
  number  = {5},
  pages   = {142},
  year    = {2025},
  publisher = {Springer},
  doi     = {10.1007/s10462-025-11129-6}
}
```

### APA
Reda, M., Onsy, A., Haikal, A. Y., & Ghanbari, A. (2025). A novel reinforcement learning-based multi-operator differential evolution with cubic spline for the path planning problem. *Artificial Intelligence Review, 58*(5), 142.

### Chicago
Reda, Mohamed, Ahmed Onsy, Amira Y. Haikal, and Ali Ghanbari. “A novel reinforcement learning-based multi-operator differential evolution with cubic spline for the path planning problem.” *Artificial Intelligence Review* 58, no. 5 (2025): 142.

---

## 📜 License

This project is released under the MIT License. See the `LICENSE` file for details.

---

## 📧 Contact

**Dr. Mohamed Reda**  
University of Central Lancashire, UK  
Mansoura University, Egypt

- 📩 Personal: [mohamed.reda.mu@gmail.com](mailto:mohamed.reda.mu@gmail.com)  
- 📩 Academic: [mohamed.reda@mans.edu.eg](mailto:mohamed.reda@mans.edu.eg)  

---

## 🌐 Academic Profiles

- 🧑‍🔬 ORCID: https://orcid.org/0000-0002-6865-1315
- 🎓 Google Scholar: https://scholar.google.com/citations?user=JmuB2qwAAAAJ
- 📊 Scopus: https://www.scopus.com/authid/detail.uri?authorId=57220204540
- 📚 Web of Science: https://www.webofscience.com/wos/author/record/3164983
- 🧾 SciProfiles: https://sciprofiles.com/profile/Mreda

---

## 🔗 Professional & Social Links

- 💼 LinkedIn: https://www.linkedin.com/in/mraf
- 🔬 ResearchGate: https://www.researchgate.net/profile/Mohamed-Reda-8
- 🎓 Academia: https://mansoura.academia.edu/MohamedRedaAboelfotohMohamed
- 📘 SciLit: https://www.scilit.net/scholars/12099081
- 🧮 MATLAB Central: https://uk.mathworks.com/matlabcentral/profile/authors/36082525
- ▶️ YouTube: https://youtube.com/@mredacs

---

## Related Repository

- QSMODE-Algorithm: https://github.com/MohamedRedaMu/QSMODE-Algorithm

---

## Acknowledgement

This repository accompanies the RP2B24 benchmark framework used in the published QSMODE paper in *Artificial Intelligence Review*.
