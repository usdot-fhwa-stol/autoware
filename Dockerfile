FROM usdotfhwastolcandidate/carma-base:vanden-plas as build

COPY --chown=carma . /home/carma/autoware.ai
RUN /home/carma/autoware.ai/docker/checkout.bash
RUN ./home/carma/autoware.ai/docker/install.sh

FROM usdotfhwastolcandidate/carma-base:vanden-plas

ARG BUILD_DATE="NULL"
ARG VCS_REF="NULL"
ARG VERSION="NULL"

LABEL org.label-schema.schema-version="1.0"
LABEL org.label-schema.name="autoware.ai"
LABEL org.label-schema.description="Binary applications and libraries from autoware.ai for the CARMA Platform"
LABEL org.label-schema.vendor="Leidos"
LABEL org.label-schema.version=${VERSION}
LABEL org.label-schema.url="https://highways.dot.gov/research/research-programs/CARMA"
LABEL org.label-schema.vcs-url="https://github.com/usdot-fhwa-stol/autoware.ai"
LABEL org.label-schema.vcs-ref=${VCS_REF}
LABEL org.label-schema.build-date=${BUILD_DATE}

COPY --chown=carma --from=build /home/carma/autoware.ai/ros/install /opt/autoware.ai/ros/install