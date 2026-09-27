# 06. GitHub 협업 규칙

## 1. 역할

| 도구 | 쓰는 곳 |
|---|---|
| GitHub | 코드, 설정, 문서. 이 저장소를 Fork 하거나 팀 저장소로 가져가 쓰고, 변경은 Pull Request 로 |
| GitHub Actions | PR 마다 자동 시험 (`알고리즘 단위시험`, `ROS 2 Jazzy 빌드/시험`) |
| GitHub Issues | 질문, 버그 보고 (`.github/ISSUE_TEMPLATE/bug_report.md` 양식) |
| 팀 채널 · 드라이브 | 진행 공유, 실험 결과(그래프·영상), rosbag · 데이터셋 · 학습 모델처럼 큰 파일 |

## 2. 브랜치와 커밋

```bash
git checkout main && git pull
git checkout -b feat/trunk-band-tuning      # feat/ fix/ docs/ exp/ 중 하나
# ... 수정 ...
./scripts/test_ws.sh                         # 시험 통과 확인
git add -A && git commit -m "perception: 줄기 검출 밴드를 품종별 파라미터로 분리"
git push -u origin feat/trunk-band-tuning
```

- 커밋 메시지: `패키지명: 무엇을 왜` (예: `navigation: U턴 반경을 행간 기준으로 계산`)
- `main` 에 직접 push 하지 않습니다. PR 을 만들고 한 명 이상 리뷰 후 merge 합니다.
- 파라미터만 바꾼 실험은 `exp/` 브랜치로 두고 결과를 PR 설명에 표로 남깁니다.
- 데이터(`data/`), rosbag 은 커밋하지 않습니다 (`.gitignore` 에 등록됨).

## 3. 결과 공유

- 하루 작업이 끝나면 팀 채널에 3줄 요약: **한 일 / 결과(숫자) / 막힌 점**
- 실험 결과는 조건(커밋, 파라미터, 월드 seed)과 지표(RMS 횡오차, 완주율)를 같이 올립니다.
- 오류 질문 양식: 실행한 명령 + 오류 전문 + 커밋 해시 (`git rev-parse --short HEAD`)

## 4. 팀 저장소 만들기 (팀장 1회)

```bash
# 이 저장소를 받아 팀 계정의 새 저장소로 올리기 (gh CLI 로그인 후)
git clone https://github.com/pathcmd95/orchard-rover.git && cd orchard-rover
gh repo create <팀 계정>/orchard-rover --private --source . --remote team --push
```

팀원 초대: 저장소 Settings → Collaborators. `main` 브랜치 보호 규칙(PR 필수, CI 통과 필수)을 켜 두는 것을 권장합니다.
