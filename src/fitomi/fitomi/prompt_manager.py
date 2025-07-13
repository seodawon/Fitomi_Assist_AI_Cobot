def get_prompt_by_command(command: str) -> str:
    # 옷 추천 관련 명령
    if any(keyword in command for keyword in ["옷", "상의", "외투", "하의", "날씨", "악세사리", "스타일"]):
        system_prompt = """
        당신은 사용자의 문장에서 다음과 같은 두 가지 옷을 추천해야 합니다.
        모든 추천은 아래 옷장 리스트를 참고해서 선택하면 됩니다.

        <목표 1> 유사한 옷 추천 
        - 1번 이미지를 기준으로 나머지 이미지 중 가장 유사한 옷을 골라주세요.(나머지 이미지는 최소 1장, 최대 4장)  
        - 2~5번 이미지는 모두 옷장 리스트에 있는 실제 옷입니다.  
        - 가장 유사한 이미지를 선택한 후, 이를 옷장에 있는 옷으로 판단해 추천하세요.
        - 1번 이미지 속 옷은 색상도 함께 말해주세요.

        <목표 2> 날씨에 맞는 옷 추천
        - 오늘 날씨 정보와 날씨별 옷차림 가이드를 참고하여 가장 어울리는 옷을 추천하세요.  

        <출력 형식>
        - [이미지 속 옷]/[유사한 옷]/[날씨에 맞는 옷]
        - 유사한 옷이 없는 경우 또는 날씨에 맞는 옷이 없는 경우 각 자리에 'x' 넣으세요.(최대한 추천 지향)
          예시)
          - 바람막이/바람막이/반팔
          - 가디건/가디건/반팔
          - 반바지/반바지/x
          - 가죽자켓/x/x
        - 정해진 단어의 출력 형식이외의 다른 말을 덧붙이지 마세요.

        <옷장 리스트>
        {closet_caption}

        <오늘 날씨>
        - 최저 기온: {min_temp}
        - 최고 기온: {max_temp}
        - 하늘 상태: {weather_pty}

        <날씨에 따른 추천 옷차림>
        {fashion_context}
        """

        return "옷장", system_prompt

    # --------------------------------------------------------------------------------
    # 식단/영양 분석
    elif any(keyword in command for keyword in ["아침", "점심", "저녁", "식단", "영양", "냉장고", "음식"]):
        system_prompt = """
        당신은 사용자의 식단에 대해 부족한 영양소를 파악하고, 이를 보충할 수 있는 음식을 추천하는 전문 영양사입니다.

        <목표>
            사용자가 제공한 음식 목록을 기반으로, 그에 따른 영양소 섭취 상태를 파악합니다.
            부족한 영양소(탄수화물, 단백질, 지방 등)를 식별합니다.
            부족한 영양소에 대해 적합한 보충 식품 하나를 추천합니다.(반드시 한 개 추천)
            음식 정보가 불충분한 경우, 이를 명확하게 언급하고 보충 식품 추천을 할 수 없습니다.
            
            만약 사용자가 "먹고 싶은 음식"을 말한 경우:
                - 그 음식이 보충 식품 리스트에 있다면 해당 음식 그대로 추천합니다.
                - 없다면, 해당 음식과 가장 유사한 영양성분을 가진 대체 식품을 추천합니다.
                - 이때 부족한 영양분은 x가 아닌 해당 음식의 가장 높은 영양성분(탄수화물, 단백질, 지방)으로 대체하여 답변합니다.

        <입력 형식>
            사용자 입력: 오늘 먹은 음식을 입력합니다. 
            보충 식품 목록: 사용자가 가지고 있는 식품 목록입니다. 이 목록 내에서만 추천 가능합니다.

        <출력 형식>
            출력은 반드시 다음과 같이 하세요:
                형식: [부족한 영양소들(,로 구분)]/[보충 식품]
                부족한 영양소들 예시: "단백질, 탄수화물"
                보충 식품 예시: "닭가슴살"
                부족한 영양소가 없다면 x/x를 출력하세요. (최대한 추천 지향)
                정해진 단어의 출력 형식이외의 다른 말을 덧붙이지 마세요.

        <사용자 입력>
        {user_input}

        <보충 식품 목록>
        {food_list}
        """

        return "냉장고", system_prompt

    # 기본 동작
    else:
        return (
            "당신은 사용자의 명령을 정중하게 응답하는 AI 어시스턴트입니다.\n\n"
            "<목표>\n"
            "- 사용자의 질문이나 요청을 최대한 성실하게 이해하고 응답하세요.\n\n"
            "<사용자 입력>\n"
            "{user_input}"
        )

########################################################################################

def generate_sentence_message(scenario_type: str, response: str, additional_data: dict = None):
    # 옷장
    if scenario_type == "옷장":
        # image_exp = response.split('/')[0]
        similar_clothes = response.split('/')[1]
        recom_clothes = response.split('/')[2]

        if recom_clothes == 'x':
            sentence = "추천드릴 옷이 없습니다."
        elif similar_clothes == recom_clothes:
            sentence = (
                f"보여주신 이미지와 유사한 옷은 {similar_clothes}입니다."
            )
        else:
            # 주인님의 옷장에서 {image_exp} 이미지와 
            sentence = (
                f"보여주신 이미지와 유사한 옷은 {similar_clothes}입니다.\n"
                f"오늘 날씨는 {additional_data['weather_pty']}이고, 기온은 최저 {additional_data['min_temp'].split('.')[0]}도, 최고 {additional_data['max_temp'].split('.')[0]}도 입니다. "
                f"따라서, {recom_clothes}도 함께 고려해보세요."
            )
        # print("@@@@@@@@@@@@@@@@@@", sentence)
        return sentence
    # 냉장고
    else:
        lack_nut = response.split('/')[0]
        supp_food = response.split('/')[1]

        if supp_food == 'x':
            sentence = "추천드릴 음식이 없습니다."
        elif lack_nut == 'x':
            sentence = "부족한 영양분이 없습니다."
        else:
            sentence = f"오늘의 부족한 영양소는 {lack_nut}입니다. {supp_food}의 섭취를 고려해보세요."
        # print("@@@@@@@@@@@@@@@@@@", sentence)
        return sentence

########################################################################################

def add_particles_to_list(words):
    def add_particle(word):
        last_char = word[-1]
        last_char_code = ord(last_char)

        if (last_char_code - 0xAC00) % 28 != 0:
            return word + "을"
        else:
            return word + "를"

    if not words:
        return ''

    if len(words) == 1:
        return add_particle(words[0])

    # 마지막 단어만 조사 붙이기
    *rest, last = words
    return ', '.join(rest) + ', ' + add_particle(last)

########################################################################################
    # 당류, /[섭취한 총 당류(g)] 
    # - 정해진 출력 형식이외의 다른 말을 덧붙이지 마세요.
    #         - 영양성분을 분석할 수 없는 경우 또는 추천할 음식이 없는 경우 각 자리에 'x' 넣으세요.
    # <식품 별 영양성분>
    # {nutrition_context}
    # - 부족한 영양소는 아래의 식품 별 영양성분를 참고하세요.
    
    # ---
    
    # 단어 입력으로 받으면 문장으로 변환해서 반환하는 함수 구현 필요
    # 입력: 단백질,지방/견과류,골뱅이 또는 바람막이/바람막이/바람막이
    # 각 문장 내 자리에 끼워넣기
    # 옷장 
    # 이미지는 A이므로, 가장 유사한 [추천 옷]을 입는 것을 추천드립니다.\n"
    # 오늘은 X, 최저 기온 Y도, 최고 기온 Z도 이므로, [추천 옷]을 입는 것을 추천드립니다.
    # 식단: 오늘은 [부족한 영양소]가 부족할 수 있습니다. 오늘 섭취한 당류는 총 [N]g입니다. [보충 식품 예시]를 고려해보세요.
    # x가 들어있는 경우에는 추천이 어렵습니다.