# proxy settings
PROXY_HOST="http://172.16.0.223:7890"
# HTTP_PROXY_ARG="--build-arg http_proxy=${PROXY_HOST}"
HTTPS_PROXY_ARG="--build-arg https_proxy=${PROXY_HOST}"

# image settings
DOCKERFILE="Dockerfile.bionic"
NAME="ct_visual_robot"
LABLE="bionic"

sudo docker build ${HTTP_PROXY_ARG} ${HTTPS_PROXY_ARG} -f ${DOCKERFILE} -t ${NAME}:${LABLE} .