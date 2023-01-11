# 개인 스터디 2023.01.11

## 깃허브 풀 리퀘스트  (git pull Request)

브렌치를 만들 저장소 컨텍
cwsfa@cwsfa:~/robot_ws/src/exploration_bot$ git remote add leeeju https://github.com/cwsfa/exploration_bot.git

cwsfa@cwsfa:~/robot_ws/src/exploration_bot$ git remote -v
leeeju  https://github.com/cwsfa/exploration_bot.git (fetch)
leeeju  https://github.com/cwsfa/exploration_bot.git (push)
origin  https://github.com/cwsfa/exploration_bot.git (fetch)
origin  https://github.com/cwsfa/exploration_bot.git (push)

cwsfa@cwsfa:~/robot_ws/src/exploration_bot$ git branch bms_test
cwsfa@cwsfa:~/robot_ws/src/exploration_bot$ git checkout bms_test 
Switched to branch 'bms_test'

cwsfa@cwsfa:~/robot_ws/src/exploration_bot$ git branch 
* bms_test
  foxy-devel
  

cwsfa@cwsfa:~/robot_ws/src/exploration_bot$ git push 
leeeju   origin

cwsfa@cwsfa:~/robot_ws/src/exploration_bot$ git commit -m bms code update
error: pathspec 'code' did not match any file(s) known to git
error: pathspec 'update' did not match any file(s) known to git

cwsfa@cwsfa:~/robot_ws/src/exploration_bot$ git add .

cwsfa@cwsfa:~/robot_ws/src/exploration_bot$ git commit -m bms code update
error: pathspec 'code' did not match any file(s) known to git
error: pathspec 'update' did not match any file(s) known to git

cwsfa@cwsfa:~/robot_ws/src/exploration_bot$ git commit -m bms test
error: pathspec 'test' did not match any file(s) known to git


cwsfa@cwsfa:~/robot_ws/src/exploration_bot$ git push 
leeeju   origin

cwsfa@cwsfa:~/robot_ws/src/exploration_bot$ git push origin 
bms_test            foxy-devel          ORIG_HEAD           origin/HEAD 
FETCH_HEAD          HEAD                origin/foxy-devel

cwsfa@cwsfa:~/robot_ws/src/exploration_bot$ git push origin bms_test 
Enumerating objects: 42, done.
Counting objects: 100% (37/37), done.
Delta compression using up to 8 threads
Compressing objects: 100% (21/21), done.
Writing objects: 100% (24/24), 2.73 KiB | 2.73 MiB/s, done.
Total 24 (delta 11), reused 0 (delta 0)
remote: Resolving deltas: 100% (11/11), completed with 6 local objects.
remote: 
remote: Create a pull request for 'bms_test' on GitHub by visiting:
remote:      https://github.com/cwsfa/exploration_bot/pull/new/bms_test
remote: 
To https://github.com/cwsfa/exploration_bot.git
 * [new branch]      bms_test -> bms_test


cwsfa@cwsfa:~/robot_ws/src/exploration_bot$ git pull 
leeeju   origin   

cwsfa@cwsfa:~/robot_ws/src/exploration_bot$ git branch -d 
bms_test     foxy-devel   

cwsfa@cwsfa:~/robot_ws/src/exploration_bot$ git branch -d bms_test 
error: Cannot delete branch 'bms_test' checked out at '/home/cwsfa/robot_ws/src/exploration_bot'

cwsfa@cwsfa:~/robot_ws/src/exploration_bot$ git config --global user.name "leeeju"
cwsfa@cwsfa:~/robot_ws/src/exploration_bot$ git config --global user.email 02stu4@gmail.com

## branch 네이밍 규칙
어떤 방식으로 브랜치의 이름을 정하는지 브랜치 종류에 따라 살펴보자.

ex1) master branch, develop branch

master와 develop 브랜치는 본래 이름 그대로 사용하는 경우가 일반적이다.

ex2) feature branch

어떤 이름도 가능하다. 단, master, develop, release-..., hotfix-... 같은 이름은 사용할 수 없다.

feature/기능요약 형식을 추천한다. ex) feature/login    <-- 기능추가

feature/{issue-number}-{feature-name} 이슈추적을 사용한다면 이와 같은 형식을 따른다.
ex) feature/1-init-project, feature/2-build-gradle-script-write

ex3) release branch

release-RB_... 또는 release-... 또는 release/...같은 이름이 일반적이다.

release-... 형식을 추천한다. ex) release-1.2

ex4) hotfix branch

hotfix-... 형식을 추천한다. ex) hotfix-1.2.1
