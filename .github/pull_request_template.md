
<!-- 

-----------------------------------
Notice: Use of AI in Pull Requests
-----------------------------------

 - Make sure authors are human users, not AI models/agents. There should not be any "Co-authored by <AI model/agent> ..." in the commits. To fix it:

   - If the PR has only one commit, do:
      1. git commit --amend
      2. Remove "Co-authored by..." line 
      3. git push origin HEAD --force-with-lease

   - In case there are commits with "Co-authored by <AI model/agent> ...", 
   squash/rebase all commits in the PR:
      1. git rebase -i origin/master
      2. Pick first commit and squash the others, example:
        pick a1b2c3d Your first commit message
        squash e4f5g6h Your second commit message
        squash i7j8k9l Your third commit message
      3. Cleanup new message of the squashed commit without any "Co-authored by..."
      4. git push --force-with-lease origin HEAD

-->

## Description
<!-- What does this change do? -->
<!-- If it fixes an existing issue, please refer to it -->



## Example
<!-- If it is a new feature or a new parameter, show an example of usage on how to test it -->



## Results
<!-- Show some results of the new feature or bug fix: expected terminal logs, a picture, gif, video... -->



## Checklist

- [ ] I have the right to submit this contribution, and I agree that it is licensed under the same [BSD 3-Clause license](https://github.com/introlab/rtabmap/blob/master/LICENSE) as the project.
- [ ] Use of AI/LLM:
  - Either I did not use any AI/LLM tool for this PR, or I named the tool/model below and reviewed all of its output myself.
  - No commit credits an AI model/agent as author or co-author.
  - I will answer maintainer questions and review comments myself, without AI/LLM.

<!-- AI/Model used: -->



### Optional
<!-- Check any that apply, they help maintainers prioritize and handle the PR, and can speed up the review process. -->

- [ ] Not urgent: maintainers can look at it whenever they have time.
- [ ] Maintainers are free to refactor this PR as much as they want, while still crediting me as a contributor.
- [ ] Inspiration only: this PR can be used as a starting point for maintainers to implement the feature their own way. I don't mind if it is not merged as is, or if my contribution doesn't appear in the commit history or the list of contributors.
