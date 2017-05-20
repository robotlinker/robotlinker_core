# stop script on error
set -e

## Check to see if root CA file exists, download if not
#if [ ! -f .keys/root-CA.crt ]; then
#  printf "\nDownloading AWS IoT Root CA certificate from Symantec...\n"
#  curl https://www.symantec.com/content/en/us/enterprise/verisign/roots/VeriSign-Class%203-Public-Primary-Certification-Authority-G5.pem > root-CA.crt
#fi
#
## install AWS Device SDK for Python if not already installed
#if [ ! -d ./aws-iot-device-sdk-python ]; then
#  printf "\nInstalling AWS SDK...\n"
#  git clone https://github.com/aws/aws-iot-device-sdk-python.git
#  pushd aws-iot-device-sdk-python
#  python setup.py install
#  popd
#fi

# run pub/sub sample app using certificates downloaded in package
printf "\nRuning pub/sub sample application...\n"
#python basicPubSub.py -e a2futgnzves0uy.iot.us-west-2.amazonaws.com -r root-CA.crt -c ljk_device.cert.pem -k ljk_device.private.key
python AWS_ROS_pubsub.py -e a2futgnzves0uy.iot.us-west-2.amazonaws.com -r ./keys/root-CA.crt -c ./keys/ljk_device.cert.pem -k ./keys/ljk_device.private.key
