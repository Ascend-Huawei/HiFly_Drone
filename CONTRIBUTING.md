# Contributing to HiFly 🛫

Welcome and thank you for your interest to contribute to HiFly Drone!

HiFly project is open-source and we welcome all your inputs! There are many ways you can contribute beyond writing code, whether it's:
- 🐛 A bug report
- 🔨 Submitting a fix
- 🗺️ Discussion on current state and system design suggestions
- ❓ Asking questions
- 💡 Proposing new features or ideas

Visit the official HiFly Drone project on GitHub: [project link](https://github.com/Ascend-Huawei/HiFly_Drone)
Before you open a new issue, please do a search in open issues to see if the issue or feature request has already been filled. If you find a relevant issue, please leave a comment and a reaction (upvote or downvote). 👍👎

<!-- #### Quick links
**[PR Submission](#submitting-a-pull-request)** <br>
**[Modules or Features Suggestions](#modules-suggestions)**<br>
**[Application Ideas](#application-ideas)**<br> -->


## Pull Request
Before we can accept a pull request (PR) from you, you will need to sign a **[Contributor License Agreement (CLA)](https://clasign.osinfra.cn/sign/Z2l0ZWUlMkZhc2NlbmQ=)**. This step takes no more than a minute and you only need to do it once. 

Be sure to follow our [Coding Guidelines](#programming-specifications) and keep code changes as small as possible
to enable us to swiftly review and accept your pull request. Please create one pull request per issue and provide the link to the issue in the pull request. You may refer to other [PR recommendations](#recommendations-for-pr-submission) below.

Submitting a pull request is simple, below we list the steps to submit a PR in 3 easy steps. 
### 1. Fork the original repo
You should fork the original repo and create personal working branches for different features/issues to keep the main repo clean and your personal workflow isolated.

### 2. Code modification
Make changes in the file to integrate your changes. Once you are done, stage, commit, and push the changes you have made to your forked repo:
```
git add .
git commit -m "commit message here"
git push -u origin
```

### 3. Preview changes and submit PR
Navigate to the original repo and submit a pull request by clicking "New Pull Request" or click on "Compare and pull request" if you are in your forked repo. Include a detail description regarding the changes you have made and then click on the "Sumbit pull request". All done, your PR is now submitted to HiFly_Drone for review and approval!

This [video](https://youtu.be/j6yU5hGP4ko) shows the process.

### Recommendations for PR Submission

To integrate your contribution as seamlessly as possible, we advise doing the follow:
- ✅ Verify your PR is **up-to-date with origin/main.** If your PR is behind origin/main, an automatic GitHub actions may be attempted by including the `/rebase` command in the comments. Alternatively, you may run the following code to fetch up-to-date contents from upstream, merge your changes, and push to your forked repo. (Replace '<local_branch>' with the name of your local branch):
```bash
git remote add upstream https://github.com/Ascend-Huawei/HiFly_Drone.git
git fetch upstream
git checkout <local_branch>
git merge upstream/main
git push -u origin
```
- ✅ Verify all Continuous Integration (CI) checks are passing.
- ✅ Make the **absolute minimal changes** required for your PR and avoid purely formatting changes when no code is modified.


## 📇 License
Before we can look at your pull request, please sign a 
The source code files must support the Apache License 2.0. You may include a statement at the beginning of each source code file as follows:
```
# Copyright 2020 Huawei Technologies Co., Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# You may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
```
If the source code contains the copyright of other companies, add a line of Huawei's copyright as follows:
```
# Copyright 2020 Huawei Technologies Co., Ltd.
# Copyright 2018 HiSillion Technologies Co., Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# You may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
```

## Programming Specifications
For Python programming, ensure your code complies with the [Python PEP 8 Coding Style](https://pep8.org/). Unit testing (UT) complies with the [pytest framework](http://www.pytest.org/en/latest/).
