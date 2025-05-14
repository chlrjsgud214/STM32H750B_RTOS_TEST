#include "RingBuffer.h"

void Ring_buffer_init(RING_BUFFER *rb)
{
	// 버퍼 내용 초기화 - 필수적인 부분만 초기화
	memset(rb->buf, 0, RING_BUFFER_SIZE); // 버퍼 내용 0으로 초기화
	rb->buf_index = 0;
	rb->ring_index = 0;
	rb->take_index = 0;

	// 인덱스 배열 초기화 - memset으로 한번에 처리
	for (int i = 0; i < MAX_BUFFER_COUNT; i++)
	{
		rb->index[i][0] = 0;
		rb->index[i][1] = 0;
		rb->index[i][2] = 0;
		rb->index[i][3] = 0;
		rb->index[i][4] = 0;
	}
}

__attribute__((always_inline)) inline int Ring_buffer_put(RING_BUFFER *rb, uint8_t data)
{
	// 데이터 저장 및 길이 증가
	rb->buf[rb->buf_index] = data;
	rb->index[rb->ring_index][LENGTH]++;

	// 버퍼 인덱스 업데이트 (조건문 최적화)
	uint16_t next_index = rb->buf_index + 1;
	if (next_index >= RING_BUFFER_SIZE)
	{
		rb->buf_index = 0;
		rb->index[rb->ring_index][OVERFLOW] = 1;
	}
	else
	{
		rb->buf_index = next_index;
	}

	return 0;
}

__attribute__((always_inline)) inline int Ring_buffer_next(RING_BUFFER *rb)
{
	// 현재 링 버퍼를 유효하게 설정
	rb->index[rb->ring_index][VALID] = 1;

	// 다음 링 버퍼 인덱스로 이동 (조건문 최적화)
	uint16_t next_ring_index = rb->ring_index + 1;
	
	if (next_ring_index == rb->take_index)
		return -1;

	if (next_ring_index >= MAX_BUFFER_COUNT)
	{
		rb->ring_index = 0;
	}
	else
	{
		rb->ring_index = next_ring_index;
	}

	// 다음 링 버퍼 초기화
	rb->index[rb->ring_index][INDEX] = rb->buf_index;
	rb->index[rb->ring_index][LENGTH] = 0;
	rb->index[rb->ring_index][USED] = 0;
	rb->index[rb->ring_index][VALID] = 0;
	rb->index[rb->ring_index][OVERFLOW] = 0;

	return 0;
}

void Ring_buffer_flush(RING_BUFFER *rb)
{
	rb->buf_index = 0;
	rb->ring_index = 0;
	rb->take_index = 0;

	// memset으로 한번에 처리
	for (int i = 0; i < MAX_BUFFER_COUNT; i++)
	{
		rb->index[i][0] = 0;
		rb->index[i][1] = 0;
		rb->index[i][2] = 0;
		rb->index[i][3] = 0;
		rb->index[i][4] = 0;
	}
}

// 패킷 가져오기 함수 (시작과 끝 인덱스 반환)
int Ring_buffer_get_packet(RING_BUFFER *rb, uint8_t *buf, uint16_t *length)
{
	// 로컬 변수로 자주 사용하는 값 캐싱
	const uint16_t take_idx = rb->take_index;
	const uint16_t idx_valid = rb->index[take_idx][VALID];
	const uint16_t idx_used = rb->index[take_idx][USED];

	// 유효한 데이터가 없거나 이미 사용된 경우 빠르게 리턴
	if (idx_valid != 1 || idx_used != 0)
	{
		*length = 0;
		return -1;
	}

	// 사용 표시
	rb->index[take_idx][USED] = 1;

	// 길이 값 캐싱
	const uint16_t packet_length = rb->index[take_idx][LENGTH];
	*length = packet_length;

	// 오버플로우 확인
	if (rb->index[take_idx][OVERFLOW] == 1)
	{
		const uint16_t start_idx = rb->index[take_idx][INDEX];
		const uint16_t overflow_length = RING_BUFFER_SIZE - 1 - start_idx;

		// 직접 버퍼에 복사 (중간 버퍼 제거)
		memcpy(buf, &rb->buf[start_idx], overflow_length);
		memcpy(buf + overflow_length, &rb->buf[0], packet_length - overflow_length);
	}
	else
	{
		// 직접 복사
		memcpy(buf, &rb->buf[rb->index[take_idx][INDEX]], packet_length);
	}

	// take_index 업데이트
	rb->take_index = (take_idx >= MAX_BUFFER_COUNT - 1) ? 0 : take_idx + 1;

	return 0; // 성공
}