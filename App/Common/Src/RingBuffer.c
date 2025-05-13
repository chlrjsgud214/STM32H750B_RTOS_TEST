#include "RingBuffer.h"

void Ring_buffer_init(RING_BUFFER *rb)
{
	for (int i = 0; i < MAX_BUFFER_COUNT; i++)
	{
		rb->index[i][0] = 0;
		rb->index[i][1] = 0;
	}
	rb->head_index = 0;
	rb->tail_index = 0;
}

int Ring_buffer_full(RING_BUFFER *rb)
{
	// 헤드와 테일의 차이가 RING_BUFFER_SIZE 이상이면 버퍼가 가득 찼음
	return (rb->index[rb->head_index][HEAD] >= RING_BUFFER_SIZE - 1)
			   ? 1
			   : 0;
}

int Ring_buffer_empty(RING_BUFFER *rb)
{
	// head_index와 tail_index가 같으면 버퍼가 비어있음
	return (rb->head_index == rb->tail_index) ? 1 : 0;
}

int Ring_buffer_put(RING_BUFFER *rb, uint8_t data)
{
	uint8_t current_head = rb->head_index & (MAX_BUFFER_COUNT - 1);

	if (Ring_buffer_full(rb) == 1)
		return -1;

	// 현재 헤드 인덱스에 해당하는 위치에 데이터 저장
	rb->buf[rb->index[current_head][HEAD] & (RING_BUFFER_SIZE - 1)] = data;

	// 현재 헤드 인덱스의 위치 증가
	rb->index[current_head][HEAD]++;

	return 0;
}

int Ring_buffer_next(RING_BUFFER *rb)
{
	if (Ring_buffer_full(rb) == 1)
		return -1;
	rb->head_index++;

	uint16_t value = rb->index[rb->head_index - 1][HEAD];
	rb->index[rb->head_index][HEAD] = value + 1;
	rb->index[rb->head_index][TAIL] = value + 1;
	return 0;
}

int Ring_buffer_get(RING_BUFFER *rb, uint8_t *data)
{
	uint8_t current_tail = rb->tail_index & (MAX_BUFFER_COUNT - 1);

	if (Ring_buffer_empty(rb) == 1)
		return -1;

	// 현재 테일 인덱스에 해당하는 위치에서 데이터 가져오기
	*data = rb->buf[rb->index[current_tail][TAIL] & (RING_BUFFER_SIZE - 1)];
	// 현재 테일 인덱스의 위치 증가
	rb->index[current_tail][TAIL]++;

	// 패킷 전체가 처리되면 테일 인덱스 증가
	if (rb->index[current_tail][TAIL] >= rb->index[current_tail][HEAD])
	{
		rb->tail_index++;
	}

	return 0;
}

void Ring_buffer_flush(RING_BUFFER *rb)
{
	// 모든 인덱스 초기화
	for (int i = 0; i < MAX_BUFFER_COUNT; i++)
	{
		rb->index[i][HEAD] = 0;
		rb->index[i][TAIL] = 0;
	}
	rb->head_index = 0;
	rb->tail_index = 0;
}

// 패킷 가져오기 함수 (시작과 끝 인덱스 반환)
int Ring_buffer_get_packet(RING_BUFFER *rb, uint16_t *start, uint16_t *end)
{
	if (Ring_buffer_empty(rb) == 1)
		return -1;

	uint8_t current_tail = rb->tail_index & (MAX_BUFFER_COUNT - 1);

	// 현재 테일 인덱스의 시작과 끝 위치 가져오기
	*end = rb->index[current_tail][HEAD];	// 현재 처리 위치
	*start = rb->index[current_tail][TAIL]; // 패킷의 끝 위치

	rb->tail_index++;

	return 0;
}

int Ring_buffer_cut_packet(RING_BUFFER *rb)
{
	if (rb->tail_index > 0)
	{
		// buffer 처리
		uint16_t buf_start = rb->index[rb->tail_index][TAIL];
		uint16_t buf_end = rb->index[rb->head_index][HEAD];
		uint16_t buf_size = buf_end - buf_start;

		if (rb->tail_index != rb->head_index)
		{
			memcpy(rb->buf,
				   &rb->buf[buf_start],
				   buf_size);
			// memset(rb->buf, 0, RING_BUFFER_SIZE);
			// memcpy(rb->buf, temp_buf, buf_size);
		}
		else
		{
			memset(rb->buf, 0, rb->index[rb->head_index][HEAD]);
		}

		// 인덱스 처리
		uint8_t active_packets = (rb->head_index - rb->tail_index) & 0xFF;

		if (active_packets != 0)
		{
			uint16_t buf_index = 0;
			// 남아있는 패킷들을 인덱스 0부터 시작하도록 이동
			for (uint8_t i = 0; i < active_packets; i++)
			{
				uint8_t old_idx = (rb->tail_index + i) & 0xFF;
				uint8_t new_idx = i & 0xFF;

				uint16_t packet_size = (rb->index[old_idx][HEAD] - rb->index[old_idx][TAIL]);

				// 인덱스 정보 복사
				uint16_t head_value = buf_index + packet_size;
				uint16_t tail_value = buf_index;
				buf_index += packet_size + 1;

				rb->index[new_idx][HEAD] = head_value;
				rb->index[new_idx][TAIL] = tail_value;
				rb->index[old_idx][HEAD] = 0;
				rb->index[old_idx][TAIL] = 0;
			}
		}
		else
		{
			uint8_t old_idx = (rb->tail_index) & 0xFF;
			uint8_t new_idx = 0;

			// 인덱스 정보 복사
			uint16_t head_value = rb->index[old_idx][HEAD] - rb->index[old_idx][TAIL];
			uint16_t tail_value = 0;

			rb->index[new_idx][HEAD] = head_value;
			rb->index[new_idx][TAIL] = tail_value;
			rb->index[old_idx][HEAD] = 0;
			rb->index[old_idx][TAIL] = 0;
		}

		// 헤드와 테일 인덱스 업데이트
		rb->head_index = active_packets;
		rb->tail_index = 0;

		return 1;
	}
	else
	{
		Ring_buffer_init(rb);
		return 0;
	}

	return 0;
}

// 버퍼 사용량 모니터링 함수 추가
uint16_t Ring_buffer_usage(RING_BUFFER *rb)
{
	if (rb->tail_index == 0)
		return 0;

	uint8_t current_head = (rb->head_index - 1) & (MAX_BUFFER_COUNT - 1);
	uint8_t current_tail = 0 & (MAX_BUFFER_COUNT - 1);

	return rb->index[current_head][HEAD] - rb->index[current_tail][TAIL];
}