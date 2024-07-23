// Benchmark "adder" written by ABC on Wed Jul 17 18:57:46 2024

module adder ( 
    \a[0] , \a[10] , \a[11] , \a[12] , \a[13] , \a[14] , \a[15] , \a[16] ,
    \a[17] , \a[18] , \a[19] , \a[1] , \a[20] , \a[21] , \a[22] , \a[23] ,
    \a[24] , \a[25] , \a[26] , \a[27] , \a[28] , \a[29] , \a[2] , \a[30] ,
    \a[31] , \a[3] , \a[4] , \a[5] , \a[6] , \a[7] , \a[8] , \a[9] ,
    \b[0] , \b[10] , \b[11] , \b[12] , \b[13] , \b[14] , \b[15] , \b[16] ,
    \b[17] , \b[18] , \b[19] , \b[1] , \b[20] , \b[21] , \b[22] , \b[23] ,
    \b[24] , \b[25] , \b[26] , \b[27] , \b[28] , \b[29] , \b[2] , \b[30] ,
    \b[3] , \b[4] , \b[5] , \b[6] , \b[7] , \b[8] , \b[9] ,
    \s[0] , \s[10] , \s[11] , \s[12] , \s[13] , \s[14] , \s[15] , \s[16] ,
    \s[17] , \s[18] , \s[19] , \s[1] , \s[20] , \s[21] , \s[22] , \s[23] ,
    \s[24] , \s[25] , \s[26] , \s[27] , \s[28] , \s[29] , \s[2] , \s[30] ,
    \s[31] , \s[3] , \s[4] , \s[5] , \s[6] , \s[7] , \s[8] , \s[9]   );
  input  \a[0] , \a[10] , \a[11] , \a[12] , \a[13] , \a[14] , \a[15] ,
    \a[16] , \a[17] , \a[18] , \a[19] , \a[1] , \a[20] , \a[21] , \a[22] ,
    \a[23] , \a[24] , \a[25] , \a[26] , \a[27] , \a[28] , \a[29] , \a[2] ,
    \a[30] , \a[31] , \a[3] , \a[4] , \a[5] , \a[6] , \a[7] , \a[8] ,
    \a[9] , \b[0] , \b[10] , \b[11] , \b[12] , \b[13] , \b[14] , \b[15] ,
    \b[16] , \b[17] , \b[18] , \b[19] , \b[1] , \b[20] , \b[21] , \b[22] ,
    \b[23] , \b[24] , \b[25] , \b[26] , \b[27] , \b[28] , \b[29] , \b[2] ,
    \b[30] , \b[3] , \b[4] , \b[5] , \b[6] , \b[7] , \b[8] , \b[9] ;
  output \s[0] , \s[10] , \s[11] , \s[12] , \s[13] , \s[14] , \s[15] ,
    \s[16] , \s[17] , \s[18] , \s[19] , \s[1] , \s[20] , \s[21] , \s[22] ,
    \s[23] , \s[24] , \s[25] , \s[26] , \s[27] , \s[28] , \s[29] , \s[2] ,
    \s[30] , \s[31] , \s[3] , \s[4] , \s[5] , \s[6] , \s[7] , \s[8] ,
    \s[9] ;
  wire new_n97, new_n98, new_n99, new_n100, new_n101, new_n102, new_n103,
    new_n104, new_n105, new_n106, new_n107, new_n108, new_n109, new_n110,
    new_n111, new_n112, new_n113, new_n114, new_n115, new_n116, new_n117,
    new_n118, new_n119, new_n120, new_n121, new_n122, new_n123, new_n124,
    new_n125, new_n126, new_n128, new_n129, new_n130, new_n131, new_n132,
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n138, new_n139,
    new_n140, new_n141, new_n143, new_n144, new_n145, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n157, new_n158, new_n159, new_n161, new_n162, new_n163, new_n164,
    new_n165, new_n166, new_n167, new_n169, new_n170, new_n171, new_n172,
    new_n173, new_n174, new_n175, new_n176, new_n177, new_n178, new_n179,
    new_n180, new_n182, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n196, new_n197, new_n198, new_n200, new_n201, new_n202, new_n203,
    new_n204, new_n205, new_n206, new_n207, new_n208, new_n209, new_n210,
    new_n211, new_n214, new_n215, new_n216, new_n217, new_n218, new_n219,
    new_n220, new_n221, new_n222, new_n224, new_n225, new_n226, new_n227,
    new_n228, new_n229, new_n230, new_n231, new_n232, new_n233, new_n234,
    new_n236, new_n237, new_n238, new_n239, new_n240, new_n241, new_n242,
    new_n243, new_n245, new_n246, new_n247, new_n248, new_n249, new_n250,
    new_n251, new_n252, new_n253, new_n254, new_n256, new_n257, new_n258,
    new_n259, new_n260, new_n261, new_n262, new_n263, new_n265, new_n266,
    new_n267, new_n268, new_n269, new_n270, new_n271, new_n272, new_n273,
    new_n275, new_n276, new_n277, new_n278, new_n279, new_n280, new_n281,
    new_n282, new_n283, new_n285, new_n286, new_n287, new_n288, new_n289,
    new_n290, new_n292, new_n293, new_n294, new_n295, new_n296, new_n297,
    new_n298, new_n299, new_n300, new_n302, new_n303, new_n304, new_n305,
    new_n306, new_n307, new_n308, new_n311, new_n312, new_n313, new_n314,
    new_n315, new_n316, new_n317, new_n319, new_n320, new_n321, new_n322,
    new_n323, new_n324, new_n325, new_n328, new_n329, new_n332, new_n334,
    new_n336;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  tech160nm_fixorc02aa1n02p5x5 g001(.a(\a[10] ), .b(\b[9] ), .out0(new_n97));
  orn002aa1n02x5               g002(.a(\a[9] ), .b(\b[8] ), .o(new_n98));
  tech160nm_finor002aa1n03p5x5 g003(.a(\b[1] ), .b(\a[2] ), .o1(new_n99));
  nand22aa1n04x5               g004(.a(\b[0] ), .b(\a[1] ), .o1(new_n100));
  nand22aa1n03x5               g005(.a(\b[1] ), .b(\a[2] ), .o1(new_n101));
  tech160nm_fiaoi012aa1n05x5   g006(.a(new_n99), .b(new_n100), .c(new_n101), .o1(new_n102));
  xnrc02aa1n02x5               g007(.a(\b[3] ), .b(\a[4] ), .out0(new_n103));
  xnrc02aa1n12x5               g008(.a(\b[2] ), .b(\a[3] ), .out0(new_n104));
  inv000aa1d42x5               g009(.a(\a[3] ), .o1(new_n105));
  nanb02aa1n12x5               g010(.a(\b[2] ), .b(new_n105), .out0(new_n106));
  oao003aa1n06x5               g011(.a(\a[4] ), .b(\b[3] ), .c(new_n106), .carry(new_n107));
  oai013aa1n06x5               g012(.a(new_n107), .b(new_n103), .c(new_n104), .d(new_n102), .o1(new_n108));
  norp02aa1n02x5               g013(.a(\b[5] ), .b(\a[6] ), .o1(new_n109));
  nanp02aa1n02x5               g014(.a(\b[5] ), .b(\a[6] ), .o1(new_n110));
  norp02aa1n03x5               g015(.a(\b[4] ), .b(\a[5] ), .o1(new_n111));
  nanp02aa1n02x5               g016(.a(\b[4] ), .b(\a[5] ), .o1(new_n112));
  nona23aa1n03x5               g017(.a(new_n112), .b(new_n110), .c(new_n109), .d(new_n111), .out0(new_n113));
  orn002aa1n24x5               g018(.a(\a[8] ), .b(\b[7] ), .o(new_n114));
  nanp02aa1n06x5               g019(.a(\b[7] ), .b(\a[8] ), .o1(new_n115));
  nanp02aa1n02x5               g020(.a(new_n114), .b(new_n115), .o1(new_n116));
  xnrc02aa1n12x5               g021(.a(\b[6] ), .b(\a[7] ), .out0(new_n117));
  nor003aa1n03x5               g022(.a(new_n113), .b(new_n116), .c(new_n117), .o1(new_n118));
  norp02aa1n02x5               g023(.a(\b[6] ), .b(\a[7] ), .o1(new_n119));
  nanp02aa1n02x5               g024(.a(new_n119), .b(new_n115), .o1(new_n120));
  oai022aa1n02x5               g025(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n121));
  nano32aa1n03x7               g026(.a(new_n117), .b(new_n121), .c(new_n110), .d(new_n115), .out0(new_n122));
  nano22aa1n12x5               g027(.a(new_n122), .b(new_n114), .c(new_n120), .out0(new_n123));
  inv000aa1n06x5               g028(.a(new_n123), .o1(new_n124));
  xorc02aa1n12x5               g029(.a(\a[9] ), .b(\b[8] ), .out0(new_n125));
  aoai13aa1n02x5               g030(.a(new_n125), .b(new_n124), .c(new_n108), .d(new_n118), .o1(new_n126));
  xnbna2aa1n03x5               g031(.a(new_n97), .b(new_n126), .c(new_n98), .out0(\s[10] ));
  nand02aa1n16x5               g032(.a(\b[9] ), .b(\a[10] ), .o1(new_n128));
  nor042aa1n02x5               g033(.a(\b[10] ), .b(\a[11] ), .o1(new_n129));
  nanp02aa1n04x5               g034(.a(\b[10] ), .b(\a[11] ), .o1(new_n130));
  norb02aa1n03x5               g035(.a(new_n130), .b(new_n129), .out0(new_n131));
  xorc02aa1n02x5               g036(.a(\a[4] ), .b(\b[3] ), .out0(new_n132));
  nona22aa1n03x5               g037(.a(new_n132), .b(new_n104), .c(new_n102), .out0(new_n133));
  norp02aa1n02x5               g038(.a(new_n117), .b(new_n116), .o1(new_n134));
  nanb02aa1n03x5               g039(.a(new_n113), .b(new_n134), .out0(new_n135));
  aoai13aa1n12x5               g040(.a(new_n123), .b(new_n135), .c(new_n133), .d(new_n107), .o1(new_n136));
  oai022aa1d18x5               g041(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n137));
  tech160nm_fiaoi012aa1n05x5   g042(.a(new_n137), .b(new_n136), .c(new_n125), .o1(new_n138));
  nano22aa1n03x7               g043(.a(new_n138), .b(new_n128), .c(new_n131), .out0(new_n139));
  inv000aa1d42x5               g044(.a(new_n128), .o1(new_n140));
  oabi12aa1n02x5               g045(.a(new_n131), .b(new_n138), .c(new_n140), .out0(new_n141));
  norb02aa1n02x5               g046(.a(new_n141), .b(new_n139), .out0(\s[11] ));
  xnrc02aa1n12x5               g047(.a(\b[11] ), .b(\a[12] ), .out0(new_n143));
  oab012aa1n02x4               g048(.a(new_n143), .b(new_n139), .c(new_n129), .out0(new_n144));
  norb03aa1n02x5               g049(.a(new_n143), .b(new_n139), .c(new_n129), .out0(new_n145));
  nor002aa1n02x5               g050(.a(new_n144), .b(new_n145), .o1(\s[12] ));
  nanp02aa1n06x5               g051(.a(new_n108), .b(new_n118), .o1(new_n147));
  xnrc02aa1n02x5               g052(.a(\b[9] ), .b(\a[10] ), .out0(new_n148));
  nona23aa1n02x4               g053(.a(new_n131), .b(new_n125), .c(new_n143), .d(new_n148), .out0(new_n149));
  norp02aa1n02x5               g054(.a(\b[11] ), .b(\a[12] ), .o1(new_n150));
  aoi112aa1n02x5               g055(.a(\b[10] ), .b(\a[11] ), .c(\a[12] ), .d(\b[11] ), .o1(new_n151));
  and002aa1n02x5               g056(.a(\b[11] ), .b(\a[12] ), .o(new_n152));
  nano23aa1n02x5               g057(.a(new_n152), .b(new_n129), .c(new_n128), .d(new_n130), .out0(new_n153));
  aoi112aa1n03x5               g058(.a(new_n151), .b(new_n150), .c(new_n153), .d(new_n137), .o1(new_n154));
  aoai13aa1n03x5               g059(.a(new_n154), .b(new_n149), .c(new_n147), .d(new_n123), .o1(new_n155));
  xorb03aa1n02x5               g060(.a(new_n155), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  nor022aa1n16x5               g061(.a(\b[12] ), .b(\a[13] ), .o1(new_n157));
  nand22aa1n04x5               g062(.a(\b[12] ), .b(\a[13] ), .o1(new_n158));
  aoi012aa1n03x5               g063(.a(new_n157), .b(new_n155), .c(new_n158), .o1(new_n159));
  xnrb03aa1n03x5               g064(.a(new_n159), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  nano32aa1n02x5               g065(.a(new_n143), .b(new_n125), .c(new_n97), .d(new_n131), .out0(new_n161));
  aoai13aa1n02x5               g066(.a(new_n161), .b(new_n124), .c(new_n108), .d(new_n118), .o1(new_n162));
  nor022aa1n08x5               g067(.a(\b[13] ), .b(\a[14] ), .o1(new_n163));
  nand22aa1n04x5               g068(.a(\b[13] ), .b(\a[14] ), .o1(new_n164));
  nona23aa1d18x5               g069(.a(new_n164), .b(new_n158), .c(new_n157), .d(new_n163), .out0(new_n165));
  aoi012aa1n02x5               g070(.a(new_n163), .b(new_n157), .c(new_n164), .o1(new_n166));
  aoai13aa1n06x5               g071(.a(new_n166), .b(new_n165), .c(new_n162), .d(new_n154), .o1(new_n167));
  xorb03aa1n02x5               g072(.a(new_n167), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  inv000aa1d42x5               g073(.a(\a[15] ), .o1(new_n169));
  nanb02aa1d36x5               g074(.a(\b[14] ), .b(new_n169), .out0(new_n170));
  inv000aa1d42x5               g075(.a(new_n165), .o1(new_n171));
  inv000aa1n02x5               g076(.a(new_n166), .o1(new_n172));
  xorc02aa1n02x5               g077(.a(\a[15] ), .b(\b[14] ), .out0(new_n173));
  aoai13aa1n02x5               g078(.a(new_n173), .b(new_n172), .c(new_n155), .d(new_n171), .o1(new_n174));
  xorc02aa1n02x5               g079(.a(\a[16] ), .b(\b[15] ), .out0(new_n175));
  inv000aa1d42x5               g080(.a(new_n175), .o1(new_n176));
  aoi012aa1n03x5               g081(.a(new_n176), .b(new_n174), .c(new_n170), .o1(new_n177));
  inv000aa1d42x5               g082(.a(new_n170), .o1(new_n178));
  nand42aa1n02x5               g083(.a(\b[14] ), .b(\a[15] ), .o1(new_n179));
  aoi112aa1n02x5               g084(.a(new_n178), .b(new_n175), .c(new_n167), .d(new_n179), .o1(new_n180));
  nor002aa1n02x5               g085(.a(new_n177), .b(new_n180), .o1(\s[16] ));
  inv000aa1d42x5               g086(.a(\a[16] ), .o1(new_n182));
  inv000aa1d42x5               g087(.a(\b[15] ), .o1(new_n183));
  oai112aa1n04x5               g088(.a(new_n170), .b(new_n179), .c(new_n183), .d(new_n182), .o1(new_n184));
  aoi112aa1n03x5               g089(.a(new_n165), .b(new_n184), .c(new_n182), .d(new_n183), .o1(new_n185));
  nanp02aa1n02x5               g090(.a(new_n161), .b(new_n185), .o1(new_n186));
  nona23aa1n02x4               g091(.a(new_n131), .b(new_n137), .c(new_n152), .d(new_n140), .out0(new_n187));
  nona22aa1n02x4               g092(.a(new_n187), .b(new_n151), .c(new_n150), .out0(new_n188));
  nanp02aa1n02x5               g093(.a(new_n183), .b(new_n182), .o1(new_n189));
  and002aa1n02x5               g094(.a(\b[15] ), .b(\a[16] ), .o(new_n190));
  aoai13aa1n02x7               g095(.a(new_n179), .b(new_n163), .c(new_n157), .d(new_n164), .o1(new_n191));
  aoai13aa1n03x5               g096(.a(new_n189), .b(new_n190), .c(new_n191), .d(new_n170), .o1(new_n192));
  tech160nm_fiaoi012aa1n05x5   g097(.a(new_n192), .b(new_n188), .c(new_n185), .o1(new_n193));
  aoai13aa1n12x5               g098(.a(new_n193), .b(new_n186), .c(new_n147), .d(new_n123), .o1(new_n194));
  xorb03aa1n02x5               g099(.a(new_n194), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  nor042aa1d18x5               g100(.a(\b[16] ), .b(\a[17] ), .o1(new_n196));
  nand02aa1d08x5               g101(.a(\b[16] ), .b(\a[17] ), .o1(new_n197));
  tech160nm_fiaoi012aa1n05x5   g102(.a(new_n196), .b(new_n194), .c(new_n197), .o1(new_n198));
  xnrb03aa1n03x5               g103(.a(new_n198), .b(\b[17] ), .c(\a[18] ), .out0(\s[18] ));
  nona22aa1n09x5               g104(.a(new_n189), .b(new_n165), .c(new_n184), .out0(new_n200));
  nor042aa1n04x5               g105(.a(new_n200), .b(new_n149), .o1(new_n201));
  oabi12aa1n12x5               g106(.a(new_n192), .b(new_n154), .c(new_n200), .out0(new_n202));
  nor042aa1n09x5               g107(.a(\b[17] ), .b(\a[18] ), .o1(new_n203));
  nand02aa1d28x5               g108(.a(\b[17] ), .b(\a[18] ), .o1(new_n204));
  nano23aa1n06x5               g109(.a(new_n196), .b(new_n203), .c(new_n204), .d(new_n197), .out0(new_n205));
  aoai13aa1n06x5               g110(.a(new_n205), .b(new_n202), .c(new_n136), .d(new_n201), .o1(new_n206));
  aoi012aa1d24x5               g111(.a(new_n203), .b(new_n196), .c(new_n204), .o1(new_n207));
  nor002aa1d32x5               g112(.a(\b[18] ), .b(\a[19] ), .o1(new_n208));
  tech160nm_finand02aa1n05x5   g113(.a(\b[18] ), .b(\a[19] ), .o1(new_n209));
  nanb02aa1n02x5               g114(.a(new_n208), .b(new_n209), .out0(new_n210));
  inv000aa1d42x5               g115(.a(new_n210), .o1(new_n211));
  xnbna2aa1n03x5               g116(.a(new_n211), .b(new_n206), .c(new_n207), .out0(\s[19] ));
  xnrc02aa1n02x5               g117(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  inv000aa1d42x5               g118(.a(new_n208), .o1(new_n214));
  inv000aa1d42x5               g119(.a(new_n207), .o1(new_n215));
  aoai13aa1n03x5               g120(.a(new_n211), .b(new_n215), .c(new_n194), .d(new_n205), .o1(new_n216));
  orn002aa1n06x5               g121(.a(\a[20] ), .b(\b[19] ), .o(new_n217));
  nand42aa1n10x5               g122(.a(\b[19] ), .b(\a[20] ), .o1(new_n218));
  nanp02aa1n02x5               g123(.a(new_n217), .b(new_n218), .o1(new_n219));
  aoi012aa1n03x5               g124(.a(new_n219), .b(new_n216), .c(new_n214), .o1(new_n220));
  aoi012aa1n02x5               g125(.a(new_n210), .b(new_n206), .c(new_n207), .o1(new_n221));
  nano22aa1n02x4               g126(.a(new_n221), .b(new_n214), .c(new_n219), .out0(new_n222));
  norp02aa1n02x5               g127(.a(new_n220), .b(new_n222), .o1(\s[20] ));
  nanb03aa1n12x5               g128(.a(new_n208), .b(new_n218), .c(new_n209), .out0(new_n224));
  inv030aa1n02x5               g129(.a(new_n224), .o1(new_n225));
  nand23aa1n04x5               g130(.a(new_n225), .b(new_n205), .c(new_n217), .o1(new_n226));
  inv000aa1d42x5               g131(.a(new_n226), .o1(new_n227));
  aoai13aa1n06x5               g132(.a(new_n227), .b(new_n202), .c(new_n136), .d(new_n201), .o1(new_n228));
  nanp02aa1n02x5               g133(.a(new_n208), .b(new_n218), .o1(new_n229));
  oai112aa1n06x5               g134(.a(new_n229), .b(new_n217), .c(new_n224), .d(new_n207), .o1(new_n230));
  inv000aa1d42x5               g135(.a(new_n230), .o1(new_n231));
  nor002aa1d32x5               g136(.a(\b[20] ), .b(\a[21] ), .o1(new_n232));
  nand02aa1n04x5               g137(.a(\b[20] ), .b(\a[21] ), .o1(new_n233));
  norb02aa1n02x5               g138(.a(new_n233), .b(new_n232), .out0(new_n234));
  xnbna2aa1n03x5               g139(.a(new_n234), .b(new_n228), .c(new_n231), .out0(\s[21] ));
  inv000aa1d42x5               g140(.a(new_n232), .o1(new_n236));
  aoai13aa1n03x5               g141(.a(new_n234), .b(new_n230), .c(new_n194), .d(new_n227), .o1(new_n237));
  nor022aa1n08x5               g142(.a(\b[21] ), .b(\a[22] ), .o1(new_n238));
  nand22aa1n04x5               g143(.a(\b[21] ), .b(\a[22] ), .o1(new_n239));
  nanb02aa1n02x5               g144(.a(new_n238), .b(new_n239), .out0(new_n240));
  aoi012aa1n03x5               g145(.a(new_n240), .b(new_n237), .c(new_n236), .o1(new_n241));
  aobi12aa1n03x5               g146(.a(new_n234), .b(new_n228), .c(new_n231), .out0(new_n242));
  nano22aa1n02x4               g147(.a(new_n242), .b(new_n236), .c(new_n240), .out0(new_n243));
  nor002aa1n02x5               g148(.a(new_n241), .b(new_n243), .o1(\s[22] ));
  nona23aa1d18x5               g149(.a(new_n239), .b(new_n233), .c(new_n232), .d(new_n238), .out0(new_n245));
  nano32aa1n02x4               g150(.a(new_n245), .b(new_n225), .c(new_n205), .d(new_n217), .out0(new_n246));
  aoai13aa1n06x5               g151(.a(new_n246), .b(new_n202), .c(new_n136), .d(new_n201), .o1(new_n247));
  inv000aa1d42x5               g152(.a(new_n245), .o1(new_n248));
  aoi012aa1n02x5               g153(.a(new_n238), .b(new_n232), .c(new_n239), .o1(new_n249));
  aobi12aa1n06x5               g154(.a(new_n249), .b(new_n230), .c(new_n248), .out0(new_n250));
  orn002aa1n24x5               g155(.a(\a[23] ), .b(\b[22] ), .o(new_n251));
  nanp02aa1n02x5               g156(.a(\b[22] ), .b(\a[23] ), .o1(new_n252));
  nanp02aa1n02x5               g157(.a(new_n251), .b(new_n252), .o1(new_n253));
  inv000aa1d42x5               g158(.a(new_n253), .o1(new_n254));
  xnbna2aa1n03x5               g159(.a(new_n254), .b(new_n247), .c(new_n250), .out0(\s[23] ));
  inv000aa1n02x5               g160(.a(new_n250), .o1(new_n256));
  aoai13aa1n03x5               g161(.a(new_n254), .b(new_n256), .c(new_n194), .d(new_n246), .o1(new_n257));
  norp02aa1n02x5               g162(.a(\b[23] ), .b(\a[24] ), .o1(new_n258));
  nanp02aa1n02x5               g163(.a(\b[23] ), .b(\a[24] ), .o1(new_n259));
  nanb02aa1n02x5               g164(.a(new_n258), .b(new_n259), .out0(new_n260));
  aoi012aa1n03x5               g165(.a(new_n260), .b(new_n257), .c(new_n251), .o1(new_n261));
  tech160nm_fiaoi012aa1n05x5   g166(.a(new_n253), .b(new_n247), .c(new_n250), .o1(new_n262));
  nano22aa1n03x7               g167(.a(new_n262), .b(new_n251), .c(new_n260), .out0(new_n263));
  norp02aa1n03x5               g168(.a(new_n261), .b(new_n263), .o1(\s[24] ));
  nanp03aa1n02x5               g169(.a(new_n251), .b(new_n252), .c(new_n259), .o1(new_n265));
  nor043aa1n04x5               g170(.a(new_n245), .b(new_n258), .c(new_n265), .o1(new_n266));
  norb02aa1n06x4               g171(.a(new_n266), .b(new_n226), .out0(new_n267));
  aoai13aa1n06x5               g172(.a(new_n267), .b(new_n202), .c(new_n136), .d(new_n201), .o1(new_n268));
  aoi112aa1n02x5               g173(.a(\b[22] ), .b(\a[23] ), .c(\a[24] ), .d(\b[23] ), .o1(new_n269));
  oai022aa1n02x5               g174(.a(new_n265), .b(new_n249), .c(\b[23] ), .d(\a[24] ), .o1(new_n270));
  aoi112aa1n06x5               g175(.a(new_n270), .b(new_n269), .c(new_n230), .d(new_n266), .o1(new_n271));
  xnrc02aa1n12x5               g176(.a(\b[24] ), .b(\a[25] ), .out0(new_n272));
  inv000aa1d42x5               g177(.a(new_n272), .o1(new_n273));
  xnbna2aa1n03x5               g178(.a(new_n273), .b(new_n268), .c(new_n271), .out0(\s[25] ));
  nor042aa1n03x5               g179(.a(\b[24] ), .b(\a[25] ), .o1(new_n275));
  inv000aa1d42x5               g180(.a(new_n275), .o1(new_n276));
  nand22aa1n03x5               g181(.a(new_n230), .b(new_n266), .o1(new_n277));
  nona22aa1n03x5               g182(.a(new_n277), .b(new_n270), .c(new_n269), .out0(new_n278));
  aoai13aa1n03x5               g183(.a(new_n273), .b(new_n278), .c(new_n194), .d(new_n267), .o1(new_n279));
  xnrc02aa1n02x5               g184(.a(\b[25] ), .b(\a[26] ), .out0(new_n280));
  aoi012aa1n03x5               g185(.a(new_n280), .b(new_n279), .c(new_n276), .o1(new_n281));
  tech160nm_fiaoi012aa1n02p5x5 g186(.a(new_n272), .b(new_n268), .c(new_n271), .o1(new_n282));
  nano22aa1n03x7               g187(.a(new_n282), .b(new_n276), .c(new_n280), .out0(new_n283));
  nor002aa1n02x5               g188(.a(new_n281), .b(new_n283), .o1(\s[26] ));
  nor042aa1n06x5               g189(.a(new_n280), .b(new_n272), .o1(new_n285));
  nano22aa1n12x5               g190(.a(new_n226), .b(new_n266), .c(new_n285), .out0(new_n286));
  aoai13aa1n06x5               g191(.a(new_n286), .b(new_n202), .c(new_n136), .d(new_n201), .o1(new_n287));
  oao003aa1n02x5               g192(.a(\a[26] ), .b(\b[25] ), .c(new_n276), .carry(new_n288));
  aobi12aa1n09x5               g193(.a(new_n288), .b(new_n278), .c(new_n285), .out0(new_n289));
  xorc02aa1n12x5               g194(.a(\a[27] ), .b(\b[26] ), .out0(new_n290));
  xnbna2aa1n03x5               g195(.a(new_n290), .b(new_n287), .c(new_n289), .out0(\s[27] ));
  norp02aa1n02x5               g196(.a(\b[26] ), .b(\a[27] ), .o1(new_n292));
  inv040aa1n03x5               g197(.a(new_n292), .o1(new_n293));
  inv000aa1d42x5               g198(.a(new_n285), .o1(new_n294));
  tech160nm_fioai012aa1n05x5   g199(.a(new_n288), .b(new_n271), .c(new_n294), .o1(new_n295));
  aoai13aa1n02x7               g200(.a(new_n290), .b(new_n295), .c(new_n194), .d(new_n286), .o1(new_n296));
  xnrc02aa1n02x5               g201(.a(\b[27] ), .b(\a[28] ), .out0(new_n297));
  aoi012aa1n03x5               g202(.a(new_n297), .b(new_n296), .c(new_n293), .o1(new_n298));
  aobi12aa1n03x5               g203(.a(new_n290), .b(new_n287), .c(new_n289), .out0(new_n299));
  nano22aa1n03x7               g204(.a(new_n299), .b(new_n293), .c(new_n297), .out0(new_n300));
  nor002aa1n02x5               g205(.a(new_n298), .b(new_n300), .o1(\s[28] ));
  norb02aa1n02x5               g206(.a(new_n290), .b(new_n297), .out0(new_n302));
  aoai13aa1n02x7               g207(.a(new_n302), .b(new_n295), .c(new_n194), .d(new_n286), .o1(new_n303));
  oao003aa1n02x5               g208(.a(\a[28] ), .b(\b[27] ), .c(new_n293), .carry(new_n304));
  xnrc02aa1n02x5               g209(.a(\b[28] ), .b(\a[29] ), .out0(new_n305));
  aoi012aa1n03x5               g210(.a(new_n305), .b(new_n303), .c(new_n304), .o1(new_n306));
  aobi12aa1n03x5               g211(.a(new_n302), .b(new_n287), .c(new_n289), .out0(new_n307));
  nano22aa1n03x7               g212(.a(new_n307), .b(new_n304), .c(new_n305), .out0(new_n308));
  nor002aa1n02x5               g213(.a(new_n306), .b(new_n308), .o1(\s[29] ));
  xorb03aa1n02x5               g214(.a(new_n100), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g215(.a(new_n290), .b(new_n305), .c(new_n297), .out0(new_n311));
  aoai13aa1n03x5               g216(.a(new_n311), .b(new_n295), .c(new_n194), .d(new_n286), .o1(new_n312));
  oao003aa1n02x5               g217(.a(\a[29] ), .b(\b[28] ), .c(new_n304), .carry(new_n313));
  xnrc02aa1n02x5               g218(.a(\b[29] ), .b(\a[30] ), .out0(new_n314));
  aoi012aa1n03x5               g219(.a(new_n314), .b(new_n312), .c(new_n313), .o1(new_n315));
  aobi12aa1n03x5               g220(.a(new_n311), .b(new_n287), .c(new_n289), .out0(new_n316));
  nano22aa1n03x7               g221(.a(new_n316), .b(new_n313), .c(new_n314), .out0(new_n317));
  nor002aa1n02x5               g222(.a(new_n315), .b(new_n317), .o1(\s[30] ));
  norb02aa1n02x5               g223(.a(new_n311), .b(new_n314), .out0(new_n319));
  aobi12aa1n03x5               g224(.a(new_n319), .b(new_n287), .c(new_n289), .out0(new_n320));
  oao003aa1n02x5               g225(.a(\a[30] ), .b(\b[29] ), .c(new_n313), .carry(new_n321));
  xnrc02aa1n02x5               g226(.a(\b[30] ), .b(\a[31] ), .out0(new_n322));
  nano22aa1n03x7               g227(.a(new_n320), .b(new_n321), .c(new_n322), .out0(new_n323));
  aoai13aa1n02x7               g228(.a(new_n319), .b(new_n295), .c(new_n194), .d(new_n286), .o1(new_n324));
  aoi012aa1n03x5               g229(.a(new_n322), .b(new_n324), .c(new_n321), .o1(new_n325));
  nor002aa1n02x5               g230(.a(new_n325), .b(new_n323), .o1(\s[31] ));
  xorb03aa1n02x5               g231(.a(new_n102), .b(\b[2] ), .c(new_n105), .out0(\s[3] ));
  orn002aa1n02x5               g232(.a(\a[4] ), .b(\b[3] ), .o(new_n328));
  oai112aa1n02x5               g233(.a(new_n106), .b(new_n103), .c(new_n104), .d(new_n102), .o1(new_n329));
  aobi12aa1n02x5               g234(.a(new_n329), .b(new_n108), .c(new_n328), .out0(\s[4] ));
  xorb03aa1n02x5               g235(.a(new_n108), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  aoi012aa1n03x5               g236(.a(new_n111), .b(new_n108), .c(new_n112), .o1(new_n332));
  xnrb03aa1n02x5               g237(.a(new_n332), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  oaoi03aa1n02x5               g238(.a(\a[6] ), .b(\b[5] ), .c(new_n332), .o1(new_n334));
  xorb03aa1n02x5               g239(.a(new_n334), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  aoib12aa1n06x5               g240(.a(new_n119), .b(new_n334), .c(new_n117), .out0(new_n336));
  xnbna2aa1n03x5               g241(.a(new_n336), .b(new_n114), .c(new_n115), .out0(\s[8] ));
  xnbna2aa1n03x5               g242(.a(new_n125), .b(new_n147), .c(new_n123), .out0(\s[9] ));
endmodule


