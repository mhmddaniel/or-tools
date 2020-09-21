#!/bin/bash
read_var() {
     VAR=$(grep -w $1 $2 | xargs)
     IFS="=" read -ra VAR <<< "$VAR"
     echo ${VAR[1]}
}
 
VERSION=$(read_var VERSION .version)
echo ${VERSION}

docker build -t registry.gitlab.com/paketid/route-optimization:dev-${VERSION} -f Dockerfile .
docker push registry.gitlab.com/paketid/route-optimization:dev-${VERSION}
docker service update --with-registry-auth --image registry.gitlab.com/paketid/route-optimization:dev-${VERSION} route-optimization-dev_app
