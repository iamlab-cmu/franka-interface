firmware_version=$1

valid_version=true
case $firmware_version in
2)
  commit=5ca631e
  ;;
3)
  commit=80197c6
  ;;
4)
  commit=f1f46fb
  ;;
5)
  commit=c452ba2
  ;;
6)
  commit=4f9e3cc
  ;;
*)
  valid_version=false
  echo "Please enter your robot's firmware version as 2, 3, 4, 5, or 6."
  ;;
esac

if [ "$valid_version" = true ] ; then
  rm -rf libfranka
  git clone https://github.com/frankaemika/libfranka.git
  cd libfranka
  git checkout $commit
  git submodule update --init --recursive
fi
