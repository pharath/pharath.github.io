# check if these files are in .gitignore
File=".gitignore"
SearchString="_site"
if grep -q $SearchString $File; then
  printf "Warning: $SearchString found in $File. No need to run this script.\n"
else
  printf "Warning: $SearchString not found in $File. Removing _site/ folder...\n"
  git rm -rf _site/ .jekyll-cache .jekyll-metadata
  git commit -m 'rebuild pages'
  git push
  printf "Warning: Removed _site/ folder.\n"

  printf "Warning: $SearchString not found in $File. Please, add $SearchString in $File.\n"
  printf "Warning: Read https://jekyllrb.com/docs/structure/\n"
fi
