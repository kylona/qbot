#include "dds/dds.h"
#include "HelloWorldData.h"
#include <stdio.h>
#include <stdlib.h>

dds_qos_t * create_readwrite_qos(
  bool ignore_local_publications
) {
  dds_duration_t ldur;
  dds_qos_t * qos = dds_create_qos();
  dds_qset_writer_data_lifecycle(qos, false); /* disable autodispose */

  dds_qset_history(qos, DDS_HISTORY_KEEP_LAST, 1);
  dds_qset_reliability(qos, DDS_RELIABILITY_RELIABLE, DDS_INFINITY);
  dds_qset_durability(qos, DDS_DURABILITY_VOLATILE);
  ldur = DDS_INFINITY;
  dds_qset_liveliness(qos, DDS_LIVELINESS_AUTOMATIC, ldur);

  if (ignore_local_publications) {
    dds_qset_ignorelocal(qos, DDS_IGNORELOCAL_PARTICIPANT);
  }
  char* user_data = "typehash=FOOBAR;";
  dds_qset_userdata(qos, user_data, 16);
  return qos;
}

int main (int argc, char ** argv)
{
  dds_entity_t participant;
  dds_entity_t topic;
  dds_entity_t writer;
  dds_return_t rc;
  dds_qos_t *qos; 
  std_msgs_msg_dds__String_ msg;
  uint32_t status = 0;
  (void)argc;
  (void)argv;

  /* Create a Participant. */
  participant = dds_create_participant (DDS_DOMAIN_DEFAULT, NULL, NULL);
  if (participant < 0)
    DDS_FATAL("dds_create_participant: %s\n", dds_strretcode(-participant));



  qos = create_readwrite_qos(true);
  /* Create a Topic. */
  topic = dds_create_topic (
    participant, &std_msgs_msg_dds__String__desc, "rt/HelloWorldData_Msg", qos, NULL);
  if (topic < 0)
    DDS_FATAL("dds_create_topic: %s\n", dds_strretcode(-topic));


  /* Create a Writer. */
  writer = dds_create_writer (participant, topic, NULL, NULL);
  if (writer < 0)
    DDS_FATAL("dds_create_writer: %s\n", dds_strretcode(-writer));

  printf("=== [Publisher]  Waiting for a reader to be discovered ...\n");
  fflush (stdout);

  rc = dds_set_status_mask(writer, DDS_PUBLICATION_MATCHED_STATUS);
  if (rc != DDS_RETCODE_OK)
    DDS_FATAL("dds_set_status_mask: %s\n", dds_strretcode(-rc));

  while(!(status & DDS_PUBLICATION_MATCHED_STATUS))
  {
    rc = dds_get_status_changes (writer, &status);
    if (rc != DDS_RETCODE_OK)
      DDS_FATAL("dds_get_status_changes: %s\n", dds_strretcode(-rc));

    /* Polling sleep. */
    dds_sleepfor (DDS_MSECS (20));
  }

  /* Create a message to write. */
  msg.data = "Hello World Baby!!";

  printf ("=== [Publisher]  Writing : ");
  printf ("Message (%s)\n", msg.data);
  fflush (stdout);

  rc = dds_write (writer, &msg);
  if (rc != DDS_RETCODE_OK)
    DDS_FATAL("dds_write: %s\n", dds_strretcode(-rc));

  /* Deleting the participant will delete all its children recursively as well. */
  rc = dds_delete (participant);
  if (rc != DDS_RETCODE_OK)
    DDS_FATAL("dds_delete: %s\n", dds_strretcode(-rc));

  dds_delete_qos(qos);

  return EXIT_SUCCESS;
}
