// Benchmark "adder" written by ABC on Wed Jul 17 23:47:52 2024

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
    new_n134, new_n135, new_n136, new_n137, new_n138, new_n139, new_n140,
    new_n141, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n160, new_n161, new_n162,
    new_n163, new_n164, new_n165, new_n167, new_n168, new_n169, new_n171,
    new_n172, new_n173, new_n174, new_n175, new_n176, new_n178, new_n179,
    new_n180, new_n181, new_n182, new_n183, new_n184, new_n185, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n195, new_n196, new_n197, new_n199, new_n200, new_n201, new_n202,
    new_n203, new_n204, new_n205, new_n206, new_n207, new_n209, new_n210,
    new_n211, new_n212, new_n213, new_n216, new_n217, new_n218, new_n219,
    new_n220, new_n221, new_n222, new_n224, new_n225, new_n226, new_n227,
    new_n228, new_n229, new_n230, new_n232, new_n233, new_n234, new_n235,
    new_n236, new_n238, new_n239, new_n240, new_n241, new_n242, new_n243,
    new_n244, new_n245, new_n246, new_n247, new_n248, new_n249, new_n251,
    new_n252, new_n253, new_n254, new_n255, new_n257, new_n258, new_n259,
    new_n260, new_n261, new_n262, new_n263, new_n264, new_n266, new_n267,
    new_n268, new_n269, new_n270, new_n272, new_n273, new_n274, new_n275,
    new_n276, new_n277, new_n278, new_n280, new_n281, new_n282, new_n283,
    new_n284, new_n285, new_n286, new_n287, new_n288, new_n289, new_n290,
    new_n291, new_n292, new_n294, new_n295, new_n296, new_n297, new_n298,
    new_n299, new_n300, new_n303, new_n304, new_n305, new_n306, new_n307,
    new_n308, new_n309, new_n311, new_n312, new_n313, new_n314, new_n315,
    new_n316, new_n317, new_n320, new_n323, new_n325, new_n327;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor002aa1n12x5               g001(.a(\b[9] ), .b(\a[10] ), .o1(new_n97));
  inv000aa1d42x5               g002(.a(new_n97), .o1(new_n98));
  nand42aa1n10x5               g003(.a(\b[9] ), .b(\a[10] ), .o1(new_n99));
  inv000aa1d42x5               g004(.a(\a[9] ), .o1(new_n100));
  inv000aa1d42x5               g005(.a(\b[8] ), .o1(new_n101));
  norp02aa1n02x5               g006(.a(\b[3] ), .b(\a[4] ), .o1(new_n102));
  nanp02aa1n02x5               g007(.a(\b[3] ), .b(\a[4] ), .o1(new_n103));
  norp02aa1n03x5               g008(.a(\b[2] ), .b(\a[3] ), .o1(new_n104));
  nanp02aa1n02x5               g009(.a(\b[2] ), .b(\a[3] ), .o1(new_n105));
  nona23aa1n03x5               g010(.a(new_n105), .b(new_n103), .c(new_n102), .d(new_n104), .out0(new_n106));
  nanp02aa1n02x5               g011(.a(\b[1] ), .b(\a[2] ), .o1(new_n107));
  nand22aa1n03x5               g012(.a(\b[0] ), .b(\a[1] ), .o1(new_n108));
  nor002aa1n02x5               g013(.a(\b[1] ), .b(\a[2] ), .o1(new_n109));
  oai012aa1n04x7               g014(.a(new_n107), .b(new_n109), .c(new_n108), .o1(new_n110));
  tech160nm_fiao0012aa1n02p5x5 g015(.a(new_n102), .b(new_n104), .c(new_n103), .o(new_n111));
  oab012aa1n06x5               g016(.a(new_n111), .b(new_n106), .c(new_n110), .out0(new_n112));
  xorc02aa1n02x5               g017(.a(\a[6] ), .b(\b[5] ), .out0(new_n113));
  nor042aa1n02x5               g018(.a(\b[7] ), .b(\a[8] ), .o1(new_n114));
  nanp02aa1n02x5               g019(.a(\b[7] ), .b(\a[8] ), .o1(new_n115));
  nanb02aa1n03x5               g020(.a(new_n114), .b(new_n115), .out0(new_n116));
  xnrc02aa1n02x5               g021(.a(\b[6] ), .b(\a[7] ), .out0(new_n117));
  xorc02aa1n02x5               g022(.a(\a[5] ), .b(\b[4] ), .out0(new_n118));
  nona23aa1n02x4               g023(.a(new_n113), .b(new_n118), .c(new_n117), .d(new_n116), .out0(new_n119));
  aoi112aa1n02x5               g024(.a(\b[6] ), .b(\a[7] ), .c(\a[8] ), .d(\b[7] ), .o1(new_n120));
  nanp02aa1n02x5               g025(.a(\b[5] ), .b(\a[6] ), .o1(new_n121));
  oai022aa1n02x5               g026(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n122));
  nano23aa1n02x4               g027(.a(new_n117), .b(new_n116), .c(new_n122), .d(new_n121), .out0(new_n123));
  norp03aa1n02x5               g028(.a(new_n123), .b(new_n120), .c(new_n114), .o1(new_n124));
  tech160nm_fioai012aa1n05x5   g029(.a(new_n124), .b(new_n112), .c(new_n119), .o1(new_n125));
  oaoi03aa1n02x5               g030(.a(new_n100), .b(new_n101), .c(new_n125), .o1(new_n126));
  xnbna2aa1n03x5               g031(.a(new_n126), .b(new_n98), .c(new_n99), .out0(\s[10] ));
  nor002aa1n02x5               g032(.a(\b[10] ), .b(\a[11] ), .o1(new_n128));
  nand42aa1n03x5               g033(.a(\b[10] ), .b(\a[11] ), .o1(new_n129));
  nanb02aa1n06x5               g034(.a(new_n128), .b(new_n129), .out0(new_n130));
  oaib12aa1n06x5               g035(.a(new_n125), .b(new_n101), .c(\a[9] ), .out0(new_n131));
  oai112aa1n06x5               g036(.a(new_n131), .b(new_n98), .c(\b[8] ), .d(\a[9] ), .o1(new_n132));
  xnbna2aa1n03x5               g037(.a(new_n130), .b(new_n132), .c(new_n99), .out0(\s[11] ));
  nor002aa1n04x5               g038(.a(\b[11] ), .b(\a[12] ), .o1(new_n134));
  nand42aa1n06x5               g039(.a(\b[11] ), .b(\a[12] ), .o1(new_n135));
  norb02aa1n02x5               g040(.a(new_n135), .b(new_n134), .out0(new_n136));
  aoi113aa1n02x5               g041(.a(new_n128), .b(new_n136), .c(new_n132), .d(new_n129), .e(new_n99), .o1(new_n137));
  inv000aa1d42x5               g042(.a(new_n99), .o1(new_n138));
  nona22aa1n02x4               g043(.a(new_n132), .b(new_n130), .c(new_n138), .out0(new_n139));
  nanb02aa1n02x5               g044(.a(new_n134), .b(new_n135), .out0(new_n140));
  oaoi13aa1n04x5               g045(.a(new_n140), .b(new_n139), .c(\a[11] ), .d(\b[10] ), .o1(new_n141));
  norp02aa1n02x5               g046(.a(new_n141), .b(new_n137), .o1(\s[12] ));
  oabi12aa1n06x5               g047(.a(new_n111), .b(new_n106), .c(new_n110), .out0(new_n143));
  norb02aa1n03x4               g048(.a(new_n115), .b(new_n114), .out0(new_n144));
  xorc02aa1n12x5               g049(.a(\a[7] ), .b(\b[6] ), .out0(new_n145));
  nanp02aa1n02x5               g050(.a(new_n145), .b(new_n144), .o1(new_n146));
  nano22aa1n03x7               g051(.a(new_n146), .b(new_n113), .c(new_n118), .out0(new_n147));
  inv000aa1n02x5               g052(.a(new_n121), .o1(new_n148));
  inv000aa1d42x5               g053(.a(\a[6] ), .o1(new_n149));
  inv000aa1d42x5               g054(.a(\b[5] ), .o1(new_n150));
  norp02aa1n02x5               g055(.a(\b[4] ), .b(\a[5] ), .o1(new_n151));
  aoi012aa1n02x5               g056(.a(new_n151), .b(new_n149), .c(new_n150), .o1(new_n152));
  nona23aa1n02x4               g057(.a(new_n145), .b(new_n144), .c(new_n152), .d(new_n148), .out0(new_n153));
  nona22aa1n02x4               g058(.a(new_n153), .b(new_n120), .c(new_n114), .out0(new_n154));
  xorc02aa1n02x5               g059(.a(\a[9] ), .b(\b[8] ), .out0(new_n155));
  norb02aa1n02x5               g060(.a(new_n99), .b(new_n97), .out0(new_n156));
  norb02aa1n02x5               g061(.a(new_n129), .b(new_n128), .out0(new_n157));
  nand23aa1n03x5               g062(.a(new_n157), .b(new_n156), .c(new_n136), .o1(new_n158));
  norb02aa1n02x5               g063(.a(new_n155), .b(new_n158), .out0(new_n159));
  aoai13aa1n02x5               g064(.a(new_n159), .b(new_n154), .c(new_n143), .d(new_n147), .o1(new_n160));
  aoi112aa1n02x5               g065(.a(\b[10] ), .b(\a[11] ), .c(\a[12] ), .d(\b[11] ), .o1(new_n161));
  oai022aa1n02x5               g066(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n162));
  nano23aa1n02x4               g067(.a(new_n140), .b(new_n130), .c(new_n162), .d(new_n99), .out0(new_n163));
  nor043aa1n02x5               g068(.a(new_n163), .b(new_n161), .c(new_n134), .o1(new_n164));
  nanp02aa1n02x5               g069(.a(new_n160), .b(new_n164), .o1(new_n165));
  xorb03aa1n02x5               g070(.a(new_n165), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  nor042aa1n04x5               g071(.a(\b[12] ), .b(\a[13] ), .o1(new_n167));
  nand42aa1d28x5               g072(.a(\b[12] ), .b(\a[13] ), .o1(new_n168));
  aoi012aa1n02x5               g073(.a(new_n167), .b(new_n165), .c(new_n168), .o1(new_n169));
  xnrb03aa1n02x5               g074(.a(new_n169), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  nor042aa1n04x5               g075(.a(\b[13] ), .b(\a[14] ), .o1(new_n171));
  nand42aa1n16x5               g076(.a(\b[13] ), .b(\a[14] ), .o1(new_n172));
  nano23aa1d15x5               g077(.a(new_n167), .b(new_n171), .c(new_n172), .d(new_n168), .out0(new_n173));
  inv000aa1d42x5               g078(.a(new_n173), .o1(new_n174));
  tech160nm_fioai012aa1n03p5x5 g079(.a(new_n172), .b(new_n171), .c(new_n167), .o1(new_n175));
  aoai13aa1n04x5               g080(.a(new_n175), .b(new_n174), .c(new_n160), .d(new_n164), .o1(new_n176));
  xorb03aa1n02x5               g081(.a(new_n176), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  nor042aa1n04x5               g082(.a(\b[14] ), .b(\a[15] ), .o1(new_n178));
  nand02aa1d04x5               g083(.a(\b[14] ), .b(\a[15] ), .o1(new_n179));
  norb02aa1n02x5               g084(.a(new_n179), .b(new_n178), .out0(new_n180));
  nor042aa1n02x5               g085(.a(\b[15] ), .b(\a[16] ), .o1(new_n181));
  nand02aa1n03x5               g086(.a(\b[15] ), .b(\a[16] ), .o1(new_n182));
  norb02aa1n02x5               g087(.a(new_n182), .b(new_n181), .out0(new_n183));
  aoi112aa1n02x5               g088(.a(new_n183), .b(new_n178), .c(new_n176), .d(new_n180), .o1(new_n184));
  aoai13aa1n04x5               g089(.a(new_n183), .b(new_n178), .c(new_n176), .d(new_n179), .o1(new_n185));
  norb02aa1n02x5               g090(.a(new_n185), .b(new_n184), .out0(\s[16] ));
  nona23aa1d18x5               g091(.a(new_n182), .b(new_n179), .c(new_n178), .d(new_n181), .out0(new_n187));
  nano23aa1n06x5               g092(.a(new_n158), .b(new_n187), .c(new_n173), .d(new_n155), .out0(new_n188));
  aoai13aa1n06x5               g093(.a(new_n188), .b(new_n154), .c(new_n143), .d(new_n147), .o1(new_n189));
  aoi012aa1n02x5               g094(.a(new_n97), .b(new_n100), .c(new_n101), .o1(new_n190));
  nona23aa1n02x4               g095(.a(new_n136), .b(new_n157), .c(new_n190), .d(new_n138), .out0(new_n191));
  nona22aa1n02x4               g096(.a(new_n191), .b(new_n161), .c(new_n134), .out0(new_n192));
  norb02aa1n15x5               g097(.a(new_n173), .b(new_n187), .out0(new_n193));
  nanp02aa1n02x5               g098(.a(new_n178), .b(new_n182), .o1(new_n194));
  oai122aa1n06x5               g099(.a(new_n194), .b(new_n187), .c(new_n175), .d(\b[15] ), .e(\a[16] ), .o1(new_n195));
  aoi012aa1n09x5               g100(.a(new_n195), .b(new_n192), .c(new_n193), .o1(new_n196));
  xorc02aa1n02x5               g101(.a(\a[17] ), .b(\b[16] ), .out0(new_n197));
  xnbna2aa1n03x5               g102(.a(new_n197), .b(new_n189), .c(new_n196), .out0(\s[17] ));
  inv000aa1d42x5               g103(.a(\a[17] ), .o1(new_n199));
  inv000aa1d42x5               g104(.a(\b[16] ), .o1(new_n200));
  nanp02aa1n02x5               g105(.a(new_n200), .b(new_n199), .o1(new_n201));
  inv020aa1n02x5               g106(.a(new_n193), .o1(new_n202));
  oabi12aa1n03x5               g107(.a(new_n195), .b(new_n164), .c(new_n202), .out0(new_n203));
  aoai13aa1n02x5               g108(.a(new_n197), .b(new_n203), .c(new_n125), .d(new_n188), .o1(new_n204));
  nor022aa1n08x5               g109(.a(\b[17] ), .b(\a[18] ), .o1(new_n205));
  nanp02aa1n04x5               g110(.a(\b[17] ), .b(\a[18] ), .o1(new_n206));
  norb02aa1n02x5               g111(.a(new_n206), .b(new_n205), .out0(new_n207));
  xnbna2aa1n03x5               g112(.a(new_n207), .b(new_n204), .c(new_n201), .out0(\s[18] ));
  nanp02aa1n02x5               g113(.a(\b[16] ), .b(\a[17] ), .o1(new_n209));
  nano32aa1d12x5               g114(.a(new_n205), .b(new_n201), .c(new_n206), .d(new_n209), .out0(new_n210));
  inv000aa1d42x5               g115(.a(new_n210), .o1(new_n211));
  aoai13aa1n09x5               g116(.a(new_n206), .b(new_n205), .c(new_n199), .d(new_n200), .o1(new_n212));
  aoai13aa1n06x5               g117(.a(new_n212), .b(new_n211), .c(new_n189), .d(new_n196), .o1(new_n213));
  xorb03aa1n02x5               g118(.a(new_n213), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g119(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor042aa1n03x5               g120(.a(\b[18] ), .b(\a[19] ), .o1(new_n216));
  nand42aa1n02x5               g121(.a(\b[18] ), .b(\a[19] ), .o1(new_n217));
  nor042aa1n03x5               g122(.a(\b[19] ), .b(\a[20] ), .o1(new_n218));
  nand42aa1n02x5               g123(.a(\b[19] ), .b(\a[20] ), .o1(new_n219));
  norb02aa1n02x5               g124(.a(new_n219), .b(new_n218), .out0(new_n220));
  aoi112aa1n03x5               g125(.a(new_n216), .b(new_n220), .c(new_n213), .d(new_n217), .o1(new_n221));
  aoai13aa1n03x5               g126(.a(new_n220), .b(new_n216), .c(new_n213), .d(new_n217), .o1(new_n222));
  norb02aa1n03x4               g127(.a(new_n222), .b(new_n221), .out0(\s[20] ));
  nano23aa1n06x5               g128(.a(new_n216), .b(new_n218), .c(new_n219), .d(new_n217), .out0(new_n224));
  nanp03aa1n02x5               g129(.a(new_n224), .b(new_n197), .c(new_n207), .o1(new_n225));
  nona23aa1n09x5               g130(.a(new_n219), .b(new_n217), .c(new_n216), .d(new_n218), .out0(new_n226));
  aoi012aa1n09x5               g131(.a(new_n218), .b(new_n216), .c(new_n219), .o1(new_n227));
  oai012aa1d24x5               g132(.a(new_n227), .b(new_n226), .c(new_n212), .o1(new_n228));
  inv000aa1d42x5               g133(.a(new_n228), .o1(new_n229));
  aoai13aa1n06x5               g134(.a(new_n229), .b(new_n225), .c(new_n189), .d(new_n196), .o1(new_n230));
  xorb03aa1n02x5               g135(.a(new_n230), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  norp02aa1n02x5               g136(.a(\b[20] ), .b(\a[21] ), .o1(new_n232));
  xorc02aa1n02x5               g137(.a(\a[21] ), .b(\b[20] ), .out0(new_n233));
  xorc02aa1n02x5               g138(.a(\a[22] ), .b(\b[21] ), .out0(new_n234));
  aoi112aa1n02x5               g139(.a(new_n232), .b(new_n234), .c(new_n230), .d(new_n233), .o1(new_n235));
  aoai13aa1n03x5               g140(.a(new_n234), .b(new_n232), .c(new_n230), .d(new_n233), .o1(new_n236));
  norb02aa1n02x7               g141(.a(new_n236), .b(new_n235), .out0(\s[22] ));
  inv000aa1d42x5               g142(.a(\a[21] ), .o1(new_n238));
  inv000aa1d42x5               g143(.a(\a[22] ), .o1(new_n239));
  xroi22aa1d04x5               g144(.a(new_n238), .b(\b[20] ), .c(new_n239), .d(\b[21] ), .out0(new_n240));
  nanp03aa1n02x5               g145(.a(new_n240), .b(new_n210), .c(new_n224), .o1(new_n241));
  inv040aa1n03x5               g146(.a(new_n212), .o1(new_n242));
  inv020aa1n02x5               g147(.a(new_n227), .o1(new_n243));
  aoai13aa1n06x5               g148(.a(new_n240), .b(new_n243), .c(new_n224), .d(new_n242), .o1(new_n244));
  inv000aa1d42x5               g149(.a(\b[21] ), .o1(new_n245));
  oaoi03aa1n09x5               g150(.a(new_n239), .b(new_n245), .c(new_n232), .o1(new_n246));
  nanp02aa1n02x5               g151(.a(new_n244), .b(new_n246), .o1(new_n247));
  inv000aa1n02x5               g152(.a(new_n247), .o1(new_n248));
  aoai13aa1n04x5               g153(.a(new_n248), .b(new_n241), .c(new_n189), .d(new_n196), .o1(new_n249));
  xorb03aa1n02x5               g154(.a(new_n249), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g155(.a(\b[22] ), .b(\a[23] ), .o1(new_n251));
  xorc02aa1n02x5               g156(.a(\a[23] ), .b(\b[22] ), .out0(new_n252));
  xorc02aa1n02x5               g157(.a(\a[24] ), .b(\b[23] ), .out0(new_n253));
  aoi112aa1n02x5               g158(.a(new_n251), .b(new_n253), .c(new_n249), .d(new_n252), .o1(new_n254));
  aoai13aa1n03x5               g159(.a(new_n253), .b(new_n251), .c(new_n249), .d(new_n252), .o1(new_n255));
  norb02aa1n02x7               g160(.a(new_n255), .b(new_n254), .out0(\s[24] ));
  and002aa1n06x5               g161(.a(new_n253), .b(new_n252), .o(new_n257));
  nanb03aa1n02x5               g162(.a(new_n225), .b(new_n257), .c(new_n240), .out0(new_n258));
  inv000aa1d42x5               g163(.a(new_n257), .o1(new_n259));
  orn002aa1n02x5               g164(.a(\a[23] ), .b(\b[22] ), .o(new_n260));
  oao003aa1n02x5               g165(.a(\a[24] ), .b(\b[23] ), .c(new_n260), .carry(new_n261));
  aoai13aa1n04x5               g166(.a(new_n261), .b(new_n259), .c(new_n244), .d(new_n246), .o1(new_n262));
  inv040aa1n03x5               g167(.a(new_n262), .o1(new_n263));
  aoai13aa1n06x5               g168(.a(new_n263), .b(new_n258), .c(new_n189), .d(new_n196), .o1(new_n264));
  xorb03aa1n02x5               g169(.a(new_n264), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g170(.a(\b[24] ), .b(\a[25] ), .o1(new_n266));
  xorc02aa1n03x5               g171(.a(\a[25] ), .b(\b[24] ), .out0(new_n267));
  tech160nm_fixorc02aa1n05x5   g172(.a(\a[26] ), .b(\b[25] ), .out0(new_n268));
  aoi112aa1n03x5               g173(.a(new_n266), .b(new_n268), .c(new_n264), .d(new_n267), .o1(new_n269));
  aoai13aa1n03x5               g174(.a(new_n268), .b(new_n266), .c(new_n264), .d(new_n267), .o1(new_n270));
  norb02aa1n03x4               g175(.a(new_n270), .b(new_n269), .out0(\s[26] ));
  and002aa1n06x5               g176(.a(new_n268), .b(new_n267), .o(new_n272));
  nano22aa1n03x7               g177(.a(new_n241), .b(new_n257), .c(new_n272), .out0(new_n273));
  aoai13aa1n06x5               g178(.a(new_n273), .b(new_n203), .c(new_n125), .d(new_n188), .o1(new_n274));
  orn002aa1n02x5               g179(.a(\a[25] ), .b(\b[24] ), .o(new_n275));
  oao003aa1n02x5               g180(.a(\a[26] ), .b(\b[25] ), .c(new_n275), .carry(new_n276));
  aobi12aa1n06x5               g181(.a(new_n276), .b(new_n262), .c(new_n272), .out0(new_n277));
  xorc02aa1n02x5               g182(.a(\a[27] ), .b(\b[26] ), .out0(new_n278));
  xnbna2aa1n03x5               g183(.a(new_n278), .b(new_n277), .c(new_n274), .out0(\s[27] ));
  norp02aa1n02x5               g184(.a(\b[26] ), .b(\a[27] ), .o1(new_n280));
  inv040aa1n03x5               g185(.a(new_n280), .o1(new_n281));
  aobi12aa1n02x5               g186(.a(new_n278), .b(new_n277), .c(new_n274), .out0(new_n282));
  xnrc02aa1n02x5               g187(.a(\b[27] ), .b(\a[28] ), .out0(new_n283));
  nano22aa1n03x5               g188(.a(new_n282), .b(new_n281), .c(new_n283), .out0(new_n284));
  inv000aa1n02x5               g189(.a(new_n273), .o1(new_n285));
  tech160nm_fiaoi012aa1n05x5   g190(.a(new_n285), .b(new_n189), .c(new_n196), .o1(new_n286));
  inv000aa1d42x5               g191(.a(new_n246), .o1(new_n287));
  aoai13aa1n06x5               g192(.a(new_n257), .b(new_n287), .c(new_n228), .d(new_n240), .o1(new_n288));
  inv000aa1d42x5               g193(.a(new_n272), .o1(new_n289));
  aoai13aa1n06x5               g194(.a(new_n276), .b(new_n289), .c(new_n288), .d(new_n261), .o1(new_n290));
  oaih12aa1n02x5               g195(.a(new_n278), .b(new_n290), .c(new_n286), .o1(new_n291));
  tech160nm_fiaoi012aa1n02p5x5 g196(.a(new_n283), .b(new_n291), .c(new_n281), .o1(new_n292));
  norp02aa1n03x5               g197(.a(new_n292), .b(new_n284), .o1(\s[28] ));
  norb02aa1n02x5               g198(.a(new_n278), .b(new_n283), .out0(new_n294));
  oaih12aa1n02x5               g199(.a(new_n294), .b(new_n290), .c(new_n286), .o1(new_n295));
  oao003aa1n02x5               g200(.a(\a[28] ), .b(\b[27] ), .c(new_n281), .carry(new_n296));
  xnrc02aa1n02x5               g201(.a(\b[28] ), .b(\a[29] ), .out0(new_n297));
  tech160nm_fiaoi012aa1n02p5x5 g202(.a(new_n297), .b(new_n295), .c(new_n296), .o1(new_n298));
  aobi12aa1n02x7               g203(.a(new_n294), .b(new_n277), .c(new_n274), .out0(new_n299));
  nano22aa1n03x5               g204(.a(new_n299), .b(new_n296), .c(new_n297), .out0(new_n300));
  norp02aa1n03x5               g205(.a(new_n298), .b(new_n300), .o1(\s[29] ));
  xorb03aa1n02x5               g206(.a(new_n108), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g207(.a(new_n278), .b(new_n297), .c(new_n283), .out0(new_n303));
  oaih12aa1n02x5               g208(.a(new_n303), .b(new_n290), .c(new_n286), .o1(new_n304));
  oao003aa1n02x5               g209(.a(\a[29] ), .b(\b[28] ), .c(new_n296), .carry(new_n305));
  xnrc02aa1n02x5               g210(.a(\b[29] ), .b(\a[30] ), .out0(new_n306));
  tech160nm_fiaoi012aa1n02p5x5 g211(.a(new_n306), .b(new_n304), .c(new_n305), .o1(new_n307));
  aobi12aa1n02x5               g212(.a(new_n303), .b(new_n277), .c(new_n274), .out0(new_n308));
  nano22aa1n03x5               g213(.a(new_n308), .b(new_n305), .c(new_n306), .out0(new_n309));
  norp02aa1n03x5               g214(.a(new_n307), .b(new_n309), .o1(\s[30] ));
  xnrc02aa1n02x5               g215(.a(\b[30] ), .b(\a[31] ), .out0(new_n311));
  norb02aa1n02x5               g216(.a(new_n303), .b(new_n306), .out0(new_n312));
  aobi12aa1n02x7               g217(.a(new_n312), .b(new_n277), .c(new_n274), .out0(new_n313));
  oao003aa1n02x5               g218(.a(\a[30] ), .b(\b[29] ), .c(new_n305), .carry(new_n314));
  nano22aa1n02x4               g219(.a(new_n313), .b(new_n311), .c(new_n314), .out0(new_n315));
  oaih12aa1n02x5               g220(.a(new_n312), .b(new_n290), .c(new_n286), .o1(new_n316));
  tech160nm_fiaoi012aa1n02p5x5 g221(.a(new_n311), .b(new_n316), .c(new_n314), .o1(new_n317));
  norp02aa1n02x5               g222(.a(new_n317), .b(new_n315), .o1(\s[31] ));
  xnrb03aa1n02x5               g223(.a(new_n110), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g224(.a(\a[3] ), .b(\b[2] ), .c(new_n110), .o1(new_n320));
  xorb03aa1n02x5               g225(.a(new_n320), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g226(.a(new_n143), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  oaoi03aa1n02x5               g227(.a(\a[5] ), .b(\b[4] ), .c(new_n112), .o1(new_n323));
  xorb03aa1n02x5               g228(.a(new_n323), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  tech160nm_fioaoi03aa1n03p5x5 g229(.a(new_n149), .b(new_n150), .c(new_n323), .o1(new_n325));
  xnrc02aa1n02x5               g230(.a(new_n325), .b(new_n145), .out0(\s[7] ));
  oaoi03aa1n02x5               g231(.a(\a[7] ), .b(\b[6] ), .c(new_n325), .o1(new_n327));
  xorb03aa1n02x5               g232(.a(new_n327), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xorb03aa1n02x5               g233(.a(new_n125), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


