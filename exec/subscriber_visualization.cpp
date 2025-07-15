#include <stdio.h>
#include <stdlib.h>
#include <dds/dds.h>
#include "tasks/utils/mocap/Keypoints.h"  // IDL 生成的结构体定义

#define TOPIC_NAME "mujoco/visualization/keypoints"

int main(void) {
    printf("[Subscriber] Starting...\n");

    dds_entity_t participant = dds_create_participant(DDS_DOMAIN_DEFAULT, NULL, NULL);
    if (participant < 0) {
        printf("Failed to create participant\n");
        return -1;
    }

    dds_entity_t topic = dds_create_topic(participant, &mujoco_visualization_KeypointsMsg_desc, TOPIC_NAME, NULL, NULL);
    if (topic < 0) {
        printf("Failed to create topic\n");
        return -1;
    }

    dds_entity_t reader = dds_create_reader(participant, topic, NULL, NULL);
    if (reader < 0) {
        printf("Failed to create reader\n");
        return -1;
    }

    printf("[Subscriber] Waiting for messages...\n");

    while (1) {
        mujoco_visualization_KeypointsMsg msg;
        void *samples[1];
        dds_sample_info_t infos[1];
        samples[0] = &msg;

        int rc = dds_take(reader, samples, infos, 1, 1);
        if (rc > 0 && infos[0].valid_data) {
            printf("[Subscriber] Received visualization:\n");
            for (int i = 0; i < 90; ++i) {
                printf("%.2f ", msg.keypoints[i]);
                if ((i + 1) % 10 == 0) printf("\n");
            }
            printf("\n");
        }

        dds_sleepfor(DDS_MSECS(100));
    }

    return 0;
}

