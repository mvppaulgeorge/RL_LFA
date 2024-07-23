// Benchmark "adder" written by ABC on Wed Jul 17 16:45:35 2024

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
    new_n125, new_n126, new_n127, new_n128, new_n129, new_n130, new_n131,
    new_n132, new_n133, new_n134, new_n136, new_n138, new_n139, new_n140,
    new_n141, new_n142, new_n143, new_n144, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n155, new_n156,
    new_n157, new_n158, new_n159, new_n160, new_n162, new_n163, new_n164,
    new_n165, new_n166, new_n167, new_n168, new_n169, new_n170, new_n171,
    new_n172, new_n174, new_n175, new_n176, new_n177, new_n178, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n192, new_n193, new_n194, new_n195,
    new_n196, new_n197, new_n198, new_n200, new_n201, new_n202, new_n203,
    new_n204, new_n205, new_n206, new_n207, new_n208, new_n209, new_n210,
    new_n211, new_n212, new_n215, new_n216, new_n217, new_n218, new_n219,
    new_n220, new_n221, new_n223, new_n224, new_n225, new_n226, new_n227,
    new_n228, new_n229, new_n231, new_n232, new_n233, new_n234, new_n235,
    new_n236, new_n238, new_n239, new_n240, new_n241, new_n242, new_n243,
    new_n244, new_n245, new_n246, new_n247, new_n249, new_n250, new_n251,
    new_n252, new_n253, new_n255, new_n256, new_n257, new_n258, new_n259,
    new_n260, new_n261, new_n262, new_n263, new_n264, new_n265, new_n266,
    new_n268, new_n269, new_n270, new_n271, new_n272, new_n274, new_n275,
    new_n276, new_n277, new_n278, new_n279, new_n280, new_n281, new_n282,
    new_n283, new_n284, new_n286, new_n287, new_n288, new_n289, new_n290,
    new_n292, new_n293, new_n294, new_n295, new_n296, new_n297, new_n298,
    new_n299, new_n302, new_n303, new_n304, new_n305, new_n306, new_n307,
    new_n308, new_n309, new_n311, new_n312, new_n313, new_n314, new_n315,
    new_n316, new_n317, new_n318, new_n319, new_n320, new_n323, new_n324,
    new_n327, new_n329, new_n331, new_n332, new_n333;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  norp02aa1n02x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  inv000aa1d42x5               g002(.a(new_n97), .o1(new_n98));
  inv040aa1d32x5               g003(.a(\a[2] ), .o1(new_n99));
  inv000aa1d42x5               g004(.a(\b[1] ), .o1(new_n100));
  nand22aa1n03x5               g005(.a(\b[0] ), .b(\a[1] ), .o1(new_n101));
  oaoi03aa1n02x5               g006(.a(new_n99), .b(new_n100), .c(new_n101), .o1(new_n102));
  nor022aa1n06x5               g007(.a(\b[3] ), .b(\a[4] ), .o1(new_n103));
  nanp02aa1n04x5               g008(.a(\b[3] ), .b(\a[4] ), .o1(new_n104));
  norp02aa1n06x5               g009(.a(\b[2] ), .b(\a[3] ), .o1(new_n105));
  nanp02aa1n02x5               g010(.a(\b[2] ), .b(\a[3] ), .o1(new_n106));
  nona23aa1n03x5               g011(.a(new_n106), .b(new_n104), .c(new_n103), .d(new_n105), .out0(new_n107));
  inv000aa1d42x5               g012(.a(\a[3] ), .o1(new_n108));
  inv000aa1d42x5               g013(.a(\b[2] ), .o1(new_n109));
  aoai13aa1n02x7               g014(.a(new_n104), .b(new_n103), .c(new_n108), .d(new_n109), .o1(new_n110));
  oai012aa1n06x5               g015(.a(new_n110), .b(new_n107), .c(new_n102), .o1(new_n111));
  nand22aa1n02x5               g016(.a(\b[5] ), .b(\a[6] ), .o1(new_n112));
  nor022aa1n06x5               g017(.a(\b[5] ), .b(\a[6] ), .o1(new_n113));
  nor022aa1n04x5               g018(.a(\b[4] ), .b(\a[5] ), .o1(new_n114));
  nanp02aa1n02x5               g019(.a(\b[4] ), .b(\a[5] ), .o1(new_n115));
  nona23aa1n02x4               g020(.a(new_n112), .b(new_n115), .c(new_n114), .d(new_n113), .out0(new_n116));
  nor022aa1n08x5               g021(.a(\b[7] ), .b(\a[8] ), .o1(new_n117));
  nanp02aa1n12x5               g022(.a(\b[7] ), .b(\a[8] ), .o1(new_n118));
  nanb02aa1n02x5               g023(.a(new_n117), .b(new_n118), .out0(new_n119));
  nor042aa1n04x5               g024(.a(\b[6] ), .b(\a[7] ), .o1(new_n120));
  nand22aa1n02x5               g025(.a(\b[6] ), .b(\a[7] ), .o1(new_n121));
  nanb02aa1n02x5               g026(.a(new_n120), .b(new_n121), .out0(new_n122));
  nor043aa1n03x5               g027(.a(new_n116), .b(new_n119), .c(new_n122), .o1(new_n123));
  nano23aa1n03x7               g028(.a(new_n117), .b(new_n120), .c(new_n121), .d(new_n118), .out0(new_n124));
  tech160nm_fiao0012aa1n02p5x5 g029(.a(new_n113), .b(new_n114), .c(new_n112), .o(new_n125));
  aoi012aa1n02x5               g030(.a(new_n117), .b(new_n120), .c(new_n118), .o1(new_n126));
  aob012aa1n06x5               g031(.a(new_n126), .b(new_n124), .c(new_n125), .out0(new_n127));
  tech160nm_fixorc02aa1n05x5   g032(.a(\a[9] ), .b(\b[8] ), .out0(new_n128));
  aoai13aa1n06x5               g033(.a(new_n128), .b(new_n127), .c(new_n111), .d(new_n123), .o1(new_n129));
  xorc02aa1n02x5               g034(.a(\a[10] ), .b(\b[9] ), .out0(new_n130));
  nanp02aa1n02x5               g035(.a(\b[9] ), .b(\a[10] ), .o1(new_n131));
  oaih22aa1d12x5               g036(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n132));
  inv000aa1d42x5               g037(.a(new_n132), .o1(new_n133));
  nanp03aa1n02x5               g038(.a(new_n129), .b(new_n131), .c(new_n133), .o1(new_n134));
  aoai13aa1n02x5               g039(.a(new_n134), .b(new_n130), .c(new_n98), .d(new_n129), .o1(\s[10] ));
  aoi022aa1n06x5               g040(.a(new_n129), .b(new_n133), .c(\b[9] ), .d(\a[10] ), .o1(new_n136));
  xorb03aa1n02x5               g041(.a(new_n136), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  nanp02aa1n12x5               g042(.a(\b[10] ), .b(\a[11] ), .o1(new_n138));
  nor042aa1n03x5               g043(.a(\b[10] ), .b(\a[11] ), .o1(new_n139));
  nor042aa1n02x5               g044(.a(\b[11] ), .b(\a[12] ), .o1(new_n140));
  nand02aa1n06x5               g045(.a(\b[11] ), .b(\a[12] ), .o1(new_n141));
  norb02aa1n02x5               g046(.a(new_n141), .b(new_n140), .out0(new_n142));
  aoi112aa1n02x5               g047(.a(new_n142), .b(new_n139), .c(new_n136), .d(new_n138), .o1(new_n143));
  aoai13aa1n02x5               g048(.a(new_n142), .b(new_n139), .c(new_n136), .d(new_n138), .o1(new_n144));
  norb02aa1n02x5               g049(.a(new_n144), .b(new_n143), .out0(\s[12] ));
  nanp02aa1n02x5               g050(.a(new_n111), .b(new_n123), .o1(new_n146));
  aobi12aa1n06x5               g051(.a(new_n126), .b(new_n124), .c(new_n125), .out0(new_n147));
  nano23aa1n03x7               g052(.a(new_n140), .b(new_n139), .c(new_n141), .d(new_n138), .out0(new_n148));
  nand23aa1n03x5               g053(.a(new_n148), .b(new_n128), .c(new_n130), .o1(new_n149));
  tech160nm_fiao0012aa1n02p5x5 g054(.a(new_n140), .b(new_n139), .c(new_n141), .o(new_n150));
  nanp02aa1n02x5               g055(.a(new_n132), .b(new_n131), .o1(new_n151));
  aoib12aa1n03x5               g056(.a(new_n150), .b(new_n148), .c(new_n151), .out0(new_n152));
  aoai13aa1n06x5               g057(.a(new_n152), .b(new_n149), .c(new_n146), .d(new_n147), .o1(new_n153));
  xorb03aa1n02x5               g058(.a(new_n153), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  xorc02aa1n12x5               g059(.a(\a[14] ), .b(\b[13] ), .out0(new_n155));
  inv000aa1d42x5               g060(.a(new_n155), .o1(new_n156));
  norp02aa1n12x5               g061(.a(\b[12] ), .b(\a[13] ), .o1(new_n157));
  tech160nm_fixorc02aa1n04x5   g062(.a(\a[13] ), .b(\b[12] ), .out0(new_n158));
  aoi112aa1n02x5               g063(.a(new_n157), .b(new_n155), .c(new_n153), .d(new_n158), .o1(new_n159));
  aoi012aa1n02x5               g064(.a(new_n157), .b(new_n153), .c(new_n158), .o1(new_n160));
  oab012aa1n02x4               g065(.a(new_n159), .b(new_n160), .c(new_n156), .out0(\s[14] ));
  and002aa1n02x5               g066(.a(new_n155), .b(new_n158), .o(new_n162));
  inv000aa1d42x5               g067(.a(\a[14] ), .o1(new_n163));
  inv000aa1d42x5               g068(.a(\b[13] ), .o1(new_n164));
  oaoi03aa1n12x5               g069(.a(new_n163), .b(new_n164), .c(new_n157), .o1(new_n165));
  inv000aa1d42x5               g070(.a(new_n165), .o1(new_n166));
  nor002aa1n03x5               g071(.a(\b[14] ), .b(\a[15] ), .o1(new_n167));
  nand42aa1n08x5               g072(.a(\b[14] ), .b(\a[15] ), .o1(new_n168));
  nanb02aa1n02x5               g073(.a(new_n167), .b(new_n168), .out0(new_n169));
  inv000aa1d42x5               g074(.a(new_n169), .o1(new_n170));
  aoai13aa1n06x5               g075(.a(new_n170), .b(new_n166), .c(new_n153), .d(new_n162), .o1(new_n171));
  aoi112aa1n02x5               g076(.a(new_n170), .b(new_n166), .c(new_n153), .d(new_n162), .o1(new_n172));
  norb02aa1n02x5               g077(.a(new_n171), .b(new_n172), .out0(\s[15] ));
  norp02aa1n04x5               g078(.a(\b[15] ), .b(\a[16] ), .o1(new_n174));
  nand42aa1n08x5               g079(.a(\b[15] ), .b(\a[16] ), .o1(new_n175));
  norb02aa1n02x5               g080(.a(new_n175), .b(new_n174), .out0(new_n176));
  aoib12aa1n02x5               g081(.a(new_n167), .b(new_n175), .c(new_n174), .out0(new_n177));
  oaih12aa1n02x5               g082(.a(new_n171), .b(\b[14] ), .c(\a[15] ), .o1(new_n178));
  aoi022aa1n02x5               g083(.a(new_n178), .b(new_n176), .c(new_n171), .d(new_n177), .o1(\s[16] ));
  nano23aa1n03x7               g084(.a(new_n167), .b(new_n174), .c(new_n175), .d(new_n168), .out0(new_n180));
  nanp03aa1d12x5               g085(.a(new_n180), .b(new_n158), .c(new_n155), .o1(new_n181));
  nor042aa1n06x5               g086(.a(new_n181), .b(new_n149), .o1(new_n182));
  aoai13aa1n06x5               g087(.a(new_n182), .b(new_n127), .c(new_n111), .d(new_n123), .o1(new_n183));
  nona23aa1n09x5               g088(.a(new_n138), .b(new_n141), .c(new_n140), .d(new_n139), .out0(new_n184));
  oabi12aa1n03x5               g089(.a(new_n150), .b(new_n184), .c(new_n151), .out0(new_n185));
  nona23aa1n02x4               g090(.a(new_n175), .b(new_n168), .c(new_n167), .d(new_n174), .out0(new_n186));
  aoi012aa1n02x5               g091(.a(new_n174), .b(new_n167), .c(new_n175), .o1(new_n187));
  tech160nm_fioai012aa1n05x5   g092(.a(new_n187), .b(new_n186), .c(new_n165), .o1(new_n188));
  aoib12aa1n12x5               g093(.a(new_n188), .b(new_n185), .c(new_n181), .out0(new_n189));
  xorc02aa1n12x5               g094(.a(\a[17] ), .b(\b[16] ), .out0(new_n190));
  xnbna2aa1n03x5               g095(.a(new_n190), .b(new_n183), .c(new_n189), .out0(\s[17] ));
  nanp02aa1n09x5               g096(.a(new_n183), .b(new_n189), .o1(new_n192));
  nor042aa1n06x5               g097(.a(\b[16] ), .b(\a[17] ), .o1(new_n193));
  nor002aa1n12x5               g098(.a(\b[17] ), .b(\a[18] ), .o1(new_n194));
  nand22aa1n09x5               g099(.a(\b[17] ), .b(\a[18] ), .o1(new_n195));
  norb02aa1n06x4               g100(.a(new_n195), .b(new_n194), .out0(new_n196));
  aoi112aa1n02x5               g101(.a(new_n193), .b(new_n196), .c(new_n192), .d(new_n190), .o1(new_n197));
  aoai13aa1n02x5               g102(.a(new_n196), .b(new_n193), .c(new_n192), .d(new_n190), .o1(new_n198));
  norb02aa1n02x5               g103(.a(new_n198), .b(new_n197), .out0(\s[18] ));
  oao003aa1n02x5               g104(.a(new_n99), .b(new_n100), .c(new_n101), .carry(new_n200));
  nano23aa1n02x4               g105(.a(new_n103), .b(new_n105), .c(new_n106), .d(new_n104), .out0(new_n201));
  aobi12aa1n02x5               g106(.a(new_n110), .b(new_n201), .c(new_n200), .out0(new_n202));
  nanb02aa1n02x5               g107(.a(new_n116), .b(new_n124), .out0(new_n203));
  tech160nm_fioai012aa1n03p5x5 g108(.a(new_n147), .b(new_n202), .c(new_n203), .o1(new_n204));
  oabi12aa1n03x5               g109(.a(new_n188), .b(new_n152), .c(new_n181), .out0(new_n205));
  and002aa1n02x5               g110(.a(new_n190), .b(new_n196), .o(new_n206));
  aoai13aa1n02x5               g111(.a(new_n206), .b(new_n205), .c(new_n204), .d(new_n182), .o1(new_n207));
  aoi012aa1d24x5               g112(.a(new_n194), .b(new_n193), .c(new_n195), .o1(new_n208));
  nor002aa1n16x5               g113(.a(\b[18] ), .b(\a[19] ), .o1(new_n209));
  nanp02aa1n04x5               g114(.a(\b[18] ), .b(\a[19] ), .o1(new_n210));
  nanb02aa1n02x5               g115(.a(new_n209), .b(new_n210), .out0(new_n211));
  inv000aa1d42x5               g116(.a(new_n211), .o1(new_n212));
  xnbna2aa1n03x5               g117(.a(new_n212), .b(new_n207), .c(new_n208), .out0(\s[19] ));
  xnrc02aa1n02x5               g118(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  inv000aa1d42x5               g119(.a(new_n208), .o1(new_n215));
  aoai13aa1n06x5               g120(.a(new_n212), .b(new_n215), .c(new_n192), .d(new_n206), .o1(new_n216));
  nor002aa1d24x5               g121(.a(\b[19] ), .b(\a[20] ), .o1(new_n217));
  nand02aa1n06x5               g122(.a(\b[19] ), .b(\a[20] ), .o1(new_n218));
  norb02aa1n02x5               g123(.a(new_n218), .b(new_n217), .out0(new_n219));
  aoib12aa1n02x5               g124(.a(new_n209), .b(new_n218), .c(new_n217), .out0(new_n220));
  tech160nm_fioai012aa1n03p5x5 g125(.a(new_n216), .b(\b[18] ), .c(\a[19] ), .o1(new_n221));
  aoi022aa1n02x7               g126(.a(new_n221), .b(new_n219), .c(new_n216), .d(new_n220), .o1(\s[20] ));
  nona23aa1d18x5               g127(.a(new_n218), .b(new_n210), .c(new_n209), .d(new_n217), .out0(new_n223));
  nano22aa1n12x5               g128(.a(new_n223), .b(new_n190), .c(new_n196), .out0(new_n224));
  aoai13aa1n06x5               g129(.a(new_n224), .b(new_n205), .c(new_n204), .d(new_n182), .o1(new_n225));
  oai012aa1n12x5               g130(.a(new_n218), .b(new_n217), .c(new_n209), .o1(new_n226));
  oai012aa1d24x5               g131(.a(new_n226), .b(new_n223), .c(new_n208), .o1(new_n227));
  inv000aa1d42x5               g132(.a(new_n227), .o1(new_n228));
  xnrc02aa1n12x5               g133(.a(\b[20] ), .b(\a[21] ), .out0(new_n229));
  xobna2aa1n03x5               g134(.a(new_n229), .b(new_n225), .c(new_n228), .out0(\s[21] ));
  tech160nm_fiao0012aa1n02p5x5 g135(.a(new_n229), .b(new_n225), .c(new_n228), .o(new_n231));
  tech160nm_fixorc02aa1n02p5x5 g136(.a(\a[22] ), .b(\b[21] ), .out0(new_n232));
  nor042aa1n06x5               g137(.a(\b[20] ), .b(\a[21] ), .o1(new_n233));
  norp02aa1n02x5               g138(.a(new_n232), .b(new_n233), .o1(new_n234));
  inv000aa1d42x5               g139(.a(new_n233), .o1(new_n235));
  aoai13aa1n02x5               g140(.a(new_n235), .b(new_n229), .c(new_n225), .d(new_n228), .o1(new_n236));
  aoi022aa1n02x5               g141(.a(new_n231), .b(new_n234), .c(new_n236), .d(new_n232), .o1(\s[22] ));
  nano23aa1n09x5               g142(.a(new_n209), .b(new_n217), .c(new_n218), .d(new_n210), .out0(new_n238));
  nanb02aa1n02x5               g143(.a(new_n229), .b(new_n232), .out0(new_n239));
  nano32aa1n02x4               g144(.a(new_n239), .b(new_n238), .c(new_n196), .d(new_n190), .out0(new_n240));
  nanp02aa1n03x5               g145(.a(new_n238), .b(new_n215), .o1(new_n241));
  oaoi03aa1n12x5               g146(.a(\a[22] ), .b(\b[21] ), .c(new_n235), .o1(new_n242));
  inv000aa1d42x5               g147(.a(new_n242), .o1(new_n243));
  aoai13aa1n03x5               g148(.a(new_n243), .b(new_n239), .c(new_n241), .d(new_n226), .o1(new_n244));
  xorc02aa1n12x5               g149(.a(\a[23] ), .b(\b[22] ), .out0(new_n245));
  aoai13aa1n06x5               g150(.a(new_n245), .b(new_n244), .c(new_n192), .d(new_n240), .o1(new_n246));
  aoi112aa1n02x5               g151(.a(new_n245), .b(new_n244), .c(new_n192), .d(new_n240), .o1(new_n247));
  norb02aa1n03x4               g152(.a(new_n246), .b(new_n247), .out0(\s[23] ));
  xorc02aa1n02x5               g153(.a(\a[24] ), .b(\b[23] ), .out0(new_n249));
  nor042aa1n06x5               g154(.a(\b[22] ), .b(\a[23] ), .o1(new_n250));
  norp02aa1n02x5               g155(.a(new_n249), .b(new_n250), .o1(new_n251));
  inv000aa1n06x5               g156(.a(new_n250), .o1(new_n252));
  nanp02aa1n03x5               g157(.a(new_n246), .b(new_n252), .o1(new_n253));
  aoi022aa1n02x7               g158(.a(new_n253), .b(new_n249), .c(new_n246), .d(new_n251), .o1(\s[24] ));
  inv000aa1n02x5               g159(.a(new_n224), .o1(new_n255));
  norb02aa1n03x5               g160(.a(new_n232), .b(new_n229), .out0(new_n256));
  xnrc02aa1n02x5               g161(.a(\b[23] ), .b(\a[24] ), .out0(new_n257));
  norb02aa1n03x5               g162(.a(new_n245), .b(new_n257), .out0(new_n258));
  nano22aa1n02x4               g163(.a(new_n255), .b(new_n256), .c(new_n258), .out0(new_n259));
  aoai13aa1n09x5               g164(.a(new_n258), .b(new_n242), .c(new_n227), .d(new_n256), .o1(new_n260));
  tech160nm_fioaoi03aa1n02p5x5 g165(.a(\a[24] ), .b(\b[23] ), .c(new_n252), .o1(new_n261));
  inv000aa1n02x5               g166(.a(new_n261), .o1(new_n262));
  nanp02aa1n02x5               g167(.a(new_n260), .b(new_n262), .o1(new_n263));
  xorc02aa1n02x5               g168(.a(\a[25] ), .b(\b[24] ), .out0(new_n264));
  aoai13aa1n06x5               g169(.a(new_n264), .b(new_n263), .c(new_n192), .d(new_n259), .o1(new_n265));
  aoi112aa1n02x5               g170(.a(new_n264), .b(new_n263), .c(new_n192), .d(new_n259), .o1(new_n266));
  norb02aa1n03x4               g171(.a(new_n265), .b(new_n266), .out0(\s[25] ));
  xorc02aa1n02x5               g172(.a(\a[26] ), .b(\b[25] ), .out0(new_n268));
  norp02aa1n02x5               g173(.a(\b[24] ), .b(\a[25] ), .o1(new_n269));
  norp02aa1n02x5               g174(.a(new_n268), .b(new_n269), .o1(new_n270));
  inv000aa1d42x5               g175(.a(\a[25] ), .o1(new_n271));
  oaib12aa1n06x5               g176(.a(new_n265), .b(\b[24] ), .c(new_n271), .out0(new_n272));
  aoi022aa1n02x7               g177(.a(new_n272), .b(new_n268), .c(new_n265), .d(new_n270), .o1(\s[26] ));
  inv040aa1d32x5               g178(.a(\a[26] ), .o1(new_n274));
  xroi22aa1d06x4               g179(.a(new_n271), .b(\b[24] ), .c(new_n274), .d(\b[25] ), .out0(new_n275));
  nano32aa1n03x7               g180(.a(new_n255), .b(new_n275), .c(new_n256), .d(new_n258), .out0(new_n276));
  aobi12aa1n12x5               g181(.a(new_n276), .b(new_n183), .c(new_n189), .out0(new_n277));
  inv000aa1d42x5               g182(.a(new_n275), .o1(new_n278));
  oai022aa1n02x5               g183(.a(\a[25] ), .b(\b[24] ), .c(\b[25] ), .d(\a[26] ), .o1(new_n279));
  oaib12aa1n02x5               g184(.a(new_n279), .b(new_n274), .c(\b[25] ), .out0(new_n280));
  aoai13aa1n12x5               g185(.a(new_n280), .b(new_n278), .c(new_n260), .d(new_n262), .o1(new_n281));
  xorc02aa1n02x5               g186(.a(\a[27] ), .b(\b[26] ), .out0(new_n282));
  tech160nm_fioai012aa1n05x5   g187(.a(new_n282), .b(new_n281), .c(new_n277), .o1(new_n283));
  aoi112aa1n02x5               g188(.a(new_n281), .b(new_n282), .c(new_n192), .d(new_n276), .o1(new_n284));
  norb02aa1n03x4               g189(.a(new_n283), .b(new_n284), .out0(\s[27] ));
  xorc02aa1n02x5               g190(.a(\a[28] ), .b(\b[27] ), .out0(new_n286));
  norp02aa1n02x5               g191(.a(\b[26] ), .b(\a[27] ), .o1(new_n287));
  norp02aa1n02x5               g192(.a(new_n286), .b(new_n287), .o1(new_n288));
  inv000aa1d42x5               g193(.a(\a[27] ), .o1(new_n289));
  oaib12aa1n06x5               g194(.a(new_n283), .b(\b[26] ), .c(new_n289), .out0(new_n290));
  aoi022aa1n02x7               g195(.a(new_n290), .b(new_n286), .c(new_n283), .d(new_n288), .o1(\s[28] ));
  inv000aa1d42x5               g196(.a(\a[28] ), .o1(new_n292));
  xroi22aa1d04x5               g197(.a(new_n289), .b(\b[26] ), .c(new_n292), .d(\b[27] ), .out0(new_n293));
  oaih12aa1n02x5               g198(.a(new_n293), .b(new_n281), .c(new_n277), .o1(new_n294));
  inv000aa1d42x5               g199(.a(\b[27] ), .o1(new_n295));
  oaoi03aa1n02x5               g200(.a(new_n292), .b(new_n295), .c(new_n287), .o1(new_n296));
  nanp02aa1n03x5               g201(.a(new_n294), .b(new_n296), .o1(new_n297));
  xorc02aa1n02x5               g202(.a(\a[29] ), .b(\b[28] ), .out0(new_n298));
  norb02aa1n02x5               g203(.a(new_n296), .b(new_n298), .out0(new_n299));
  aoi022aa1n03x5               g204(.a(new_n297), .b(new_n298), .c(new_n294), .d(new_n299), .o1(\s[29] ));
  xorb03aa1n02x5               g205(.a(new_n101), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  and003aa1n02x5               g206(.a(new_n282), .b(new_n298), .c(new_n286), .o(new_n302));
  tech160nm_fioai012aa1n04x5   g207(.a(new_n302), .b(new_n281), .c(new_n277), .o1(new_n303));
  oao003aa1n02x5               g208(.a(\a[29] ), .b(\b[28] ), .c(new_n296), .carry(new_n304));
  nanp02aa1n03x5               g209(.a(new_n303), .b(new_n304), .o1(new_n305));
  xorc02aa1n02x5               g210(.a(\a[30] ), .b(\b[29] ), .out0(new_n306));
  aoi012aa1n02x5               g211(.a(new_n296), .b(\a[29] ), .c(\b[28] ), .o1(new_n307));
  oabi12aa1n02x5               g212(.a(new_n306), .b(\a[29] ), .c(\b[28] ), .out0(new_n308));
  norp02aa1n02x5               g213(.a(new_n308), .b(new_n307), .o1(new_n309));
  aoi022aa1n02x5               g214(.a(new_n305), .b(new_n306), .c(new_n303), .d(new_n309), .o1(\s[30] ));
  xnrc02aa1n02x5               g215(.a(\b[30] ), .b(\a[31] ), .out0(new_n311));
  and003aa1n02x5               g216(.a(new_n293), .b(new_n306), .c(new_n298), .o(new_n312));
  oaih12aa1n02x5               g217(.a(new_n312), .b(new_n281), .c(new_n277), .o1(new_n313));
  oao003aa1n02x5               g218(.a(\a[30] ), .b(\b[29] ), .c(new_n304), .carry(new_n314));
  tech160nm_fiaoi012aa1n02p5x5 g219(.a(new_n311), .b(new_n313), .c(new_n314), .o1(new_n315));
  aoai13aa1n02x5               g220(.a(new_n276), .b(new_n205), .c(new_n204), .d(new_n182), .o1(new_n316));
  aoai13aa1n03x5               g221(.a(new_n275), .b(new_n261), .c(new_n244), .d(new_n258), .o1(new_n317));
  inv000aa1d42x5               g222(.a(new_n312), .o1(new_n318));
  aoi013aa1n02x4               g223(.a(new_n318), .b(new_n316), .c(new_n317), .d(new_n280), .o1(new_n319));
  nano22aa1n02x4               g224(.a(new_n319), .b(new_n311), .c(new_n314), .out0(new_n320));
  norp02aa1n03x5               g225(.a(new_n315), .b(new_n320), .o1(\s[31] ));
  xorb03aa1n02x5               g226(.a(new_n102), .b(\b[2] ), .c(new_n108), .out0(\s[3] ));
  nanb03aa1n02x5               g227(.a(new_n105), .b(new_n200), .c(new_n106), .out0(new_n323));
  aboi22aa1n03x5               g228(.a(new_n103), .b(new_n104), .c(new_n108), .d(new_n109), .out0(new_n324));
  aboi22aa1n03x5               g229(.a(new_n103), .b(new_n111), .c(new_n323), .d(new_n324), .out0(\s[4] ));
  xorb03aa1n02x5               g230(.a(new_n111), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  oaoi03aa1n02x5               g231(.a(\a[5] ), .b(\b[4] ), .c(new_n202), .o1(new_n327));
  xorb03aa1n02x5               g232(.a(new_n327), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  aoi012aa1n02x5               g233(.a(new_n113), .b(new_n327), .c(new_n112), .o1(new_n329));
  xnrb03aa1n02x5               g234(.a(new_n329), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  oaoi03aa1n02x5               g235(.a(\a[7] ), .b(\b[6] ), .c(new_n329), .o1(new_n331));
  obai22aa1n02x7               g236(.a(new_n118), .b(new_n117), .c(\a[7] ), .d(\b[6] ), .out0(new_n332));
  oab012aa1n02x4               g237(.a(new_n332), .b(new_n329), .c(new_n122), .out0(new_n333));
  aoib12aa1n02x5               g238(.a(new_n333), .b(new_n331), .c(new_n119), .out0(\s[8] ));
  xnbna2aa1n03x5               g239(.a(new_n128), .b(new_n146), .c(new_n147), .out0(\s[9] ));
endmodule


