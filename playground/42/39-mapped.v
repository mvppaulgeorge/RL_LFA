// Benchmark "adder" written by ABC on Thu Jul 18 09:50:28 2024

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
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n156,
    new_n157, new_n159, new_n160, new_n161, new_n162, new_n163, new_n164,
    new_n165, new_n167, new_n168, new_n169, new_n170, new_n171, new_n172,
    new_n173, new_n175, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n186, new_n188,
    new_n189, new_n190, new_n191, new_n192, new_n194, new_n195, new_n196,
    new_n197, new_n198, new_n199, new_n200, new_n201, new_n202, new_n203,
    new_n204, new_n207, new_n208, new_n209, new_n210, new_n211, new_n212,
    new_n214, new_n215, new_n216, new_n217, new_n218, new_n219, new_n220,
    new_n221, new_n222, new_n224, new_n225, new_n226, new_n227, new_n228,
    new_n229, new_n230, new_n232, new_n233, new_n234, new_n235, new_n236,
    new_n237, new_n238, new_n239, new_n240, new_n241, new_n242, new_n243,
    new_n244, new_n246, new_n247, new_n248, new_n249, new_n250, new_n251,
    new_n252, new_n253, new_n254, new_n256, new_n257, new_n258, new_n259,
    new_n260, new_n261, new_n262, new_n263, new_n264, new_n265, new_n267,
    new_n268, new_n269, new_n270, new_n271, new_n272, new_n273, new_n274,
    new_n276, new_n277, new_n278, new_n279, new_n280, new_n281, new_n282,
    new_n283, new_n284, new_n285, new_n286, new_n287, new_n288, new_n289,
    new_n291, new_n292, new_n293, new_n294, new_n295, new_n296, new_n297,
    new_n299, new_n300, new_n301, new_n302, new_n303, new_n304, new_n305,
    new_n306, new_n309, new_n310, new_n311, new_n312, new_n313, new_n314,
    new_n315, new_n316, new_n318, new_n319, new_n320, new_n321, new_n322,
    new_n323, new_n324, new_n325, new_n327, new_n329, new_n331, new_n333,
    new_n334, new_n335, new_n336, new_n339, new_n340;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nanp02aa1n02x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  xorc02aa1n12x5               g002(.a(\a[9] ), .b(\b[8] ), .out0(new_n98));
  nanp02aa1n04x5               g003(.a(\b[0] ), .b(\a[1] ), .o1(new_n99));
  nand42aa1n04x5               g004(.a(\b[1] ), .b(\a[2] ), .o1(new_n100));
  nor042aa1n02x5               g005(.a(\b[1] ), .b(\a[2] ), .o1(new_n101));
  nor042aa1n06x5               g006(.a(\b[2] ), .b(\a[3] ), .o1(new_n102));
  nand02aa1d16x5               g007(.a(\b[2] ), .b(\a[3] ), .o1(new_n103));
  norb02aa1n15x5               g008(.a(new_n103), .b(new_n102), .out0(new_n104));
  oai112aa1n06x5               g009(.a(new_n104), .b(new_n100), .c(new_n101), .d(new_n99), .o1(new_n105));
  oab012aa1n02x4               g010(.a(new_n102), .b(\a[4] ), .c(\b[3] ), .out0(new_n106));
  nand22aa1n04x5               g011(.a(new_n105), .b(new_n106), .o1(new_n107));
  nand42aa1n04x5               g012(.a(\b[7] ), .b(\a[8] ), .o1(new_n108));
  nor002aa1d32x5               g013(.a(\b[7] ), .b(\a[8] ), .o1(new_n109));
  nor002aa1n16x5               g014(.a(\b[4] ), .b(\a[5] ), .o1(new_n110));
  nanp02aa1n04x5               g015(.a(\b[4] ), .b(\a[5] ), .o1(new_n111));
  nona23aa1n09x5               g016(.a(new_n108), .b(new_n111), .c(new_n110), .d(new_n109), .out0(new_n112));
  xnrc02aa1n12x5               g017(.a(\b[6] ), .b(\a[7] ), .out0(new_n113));
  nor002aa1n16x5               g018(.a(\b[5] ), .b(\a[6] ), .o1(new_n114));
  nand42aa1n10x5               g019(.a(\b[5] ), .b(\a[6] ), .o1(new_n115));
  nanp02aa1n04x5               g020(.a(\b[3] ), .b(\a[4] ), .o1(new_n116));
  nanb03aa1n03x5               g021(.a(new_n114), .b(new_n116), .c(new_n115), .out0(new_n117));
  nor043aa1n03x5               g022(.a(new_n112), .b(new_n113), .c(new_n117), .o1(new_n118));
  nanp02aa1n03x5               g023(.a(new_n118), .b(new_n107), .o1(new_n119));
  orn002aa1n02x5               g024(.a(\a[7] ), .b(\b[6] ), .o(new_n120));
  tech160nm_fioai012aa1n05x5   g025(.a(new_n115), .b(new_n114), .c(new_n110), .o1(new_n121));
  aob012aa1n02x5               g026(.a(new_n108), .b(\b[6] ), .c(\a[7] ), .out0(new_n122));
  aoi012aa1n02x5               g027(.a(new_n122), .b(new_n121), .c(new_n120), .o1(new_n123));
  nor002aa1n02x5               g028(.a(new_n123), .b(new_n109), .o1(new_n124));
  nanp03aa1n02x5               g029(.a(new_n119), .b(new_n98), .c(new_n124), .o1(new_n125));
  xorc02aa1n12x5               g030(.a(\a[10] ), .b(\b[9] ), .out0(new_n126));
  xobna2aa1n03x5               g031(.a(new_n126), .b(new_n125), .c(new_n97), .out0(\s[10] ));
  nand22aa1n09x5               g032(.a(\b[9] ), .b(\a[10] ), .o1(new_n128));
  aob012aa1n03x5               g033(.a(new_n126), .b(new_n125), .c(new_n97), .out0(new_n129));
  nor002aa1d32x5               g034(.a(\b[10] ), .b(\a[11] ), .o1(new_n130));
  nand42aa1n06x5               g035(.a(\b[10] ), .b(\a[11] ), .o1(new_n131));
  nanb02aa1n06x5               g036(.a(new_n130), .b(new_n131), .out0(new_n132));
  xnbna2aa1n03x5               g037(.a(new_n132), .b(new_n129), .c(new_n128), .out0(\s[11] ));
  inv030aa1n06x5               g038(.a(new_n130), .o1(new_n134));
  inv000aa1d42x5               g039(.a(new_n128), .o1(new_n135));
  nona22aa1n02x5               g040(.a(new_n129), .b(new_n132), .c(new_n135), .out0(new_n136));
  nor042aa1n04x5               g041(.a(\b[11] ), .b(\a[12] ), .o1(new_n137));
  tech160nm_finand02aa1n05x5   g042(.a(\b[11] ), .b(\a[12] ), .o1(new_n138));
  nanb02aa1n06x5               g043(.a(new_n137), .b(new_n138), .out0(new_n139));
  aoi012aa1n02x5               g044(.a(new_n139), .b(new_n136), .c(new_n134), .o1(new_n140));
  nanp03aa1n02x5               g045(.a(new_n136), .b(new_n134), .c(new_n139), .o1(new_n141));
  norb02aa1n02x7               g046(.a(new_n141), .b(new_n140), .out0(\s[12] ));
  inv000aa1d42x5               g047(.a(new_n109), .o1(new_n143));
  aoai13aa1n06x5               g048(.a(new_n143), .b(new_n122), .c(new_n121), .d(new_n120), .o1(new_n144));
  nano23aa1n06x5               g049(.a(new_n130), .b(new_n137), .c(new_n138), .d(new_n131), .out0(new_n145));
  and003aa1n02x5               g050(.a(new_n145), .b(new_n126), .c(new_n98), .o(new_n146));
  aoai13aa1n06x5               g051(.a(new_n146), .b(new_n144), .c(new_n118), .d(new_n107), .o1(new_n147));
  oai022aa1n02x5               g052(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n148));
  nano23aa1n09x5               g053(.a(new_n139), .b(new_n132), .c(new_n148), .d(new_n128), .out0(new_n149));
  oaoi03aa1n06x5               g054(.a(\a[12] ), .b(\b[11] ), .c(new_n134), .o1(new_n150));
  norp02aa1n06x5               g055(.a(new_n149), .b(new_n150), .o1(new_n151));
  nor042aa1d18x5               g056(.a(\b[12] ), .b(\a[13] ), .o1(new_n152));
  nand02aa1d10x5               g057(.a(\b[12] ), .b(\a[13] ), .o1(new_n153));
  nanb02aa1n02x5               g058(.a(new_n152), .b(new_n153), .out0(new_n154));
  xobna2aa1n03x5               g059(.a(new_n154), .b(new_n147), .c(new_n151), .out0(\s[13] ));
  inv000aa1d42x5               g060(.a(new_n152), .o1(new_n156));
  aoai13aa1n02x5               g061(.a(new_n156), .b(new_n154), .c(new_n147), .d(new_n151), .o1(new_n157));
  xorb03aa1n02x5               g062(.a(new_n157), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  nand02aa1d16x5               g063(.a(\b[13] ), .b(\a[14] ), .o1(new_n159));
  nor002aa1n12x5               g064(.a(\b[14] ), .b(\a[15] ), .o1(new_n160));
  nand02aa1d24x5               g065(.a(\b[14] ), .b(\a[15] ), .o1(new_n161));
  norb02aa1n02x5               g066(.a(new_n161), .b(new_n160), .out0(new_n162));
  nor042aa1n04x5               g067(.a(\b[13] ), .b(\a[14] ), .o1(new_n163));
  norp02aa1n02x5               g068(.a(new_n163), .b(new_n152), .o1(new_n164));
  aoai13aa1n04x5               g069(.a(new_n164), .b(new_n154), .c(new_n147), .d(new_n151), .o1(new_n165));
  xobna2aa1n03x5               g070(.a(new_n162), .b(new_n165), .c(new_n159), .out0(\s[15] ));
  nanp03aa1n02x5               g071(.a(new_n165), .b(new_n159), .c(new_n162), .o1(new_n167));
  nor002aa1n16x5               g072(.a(\b[15] ), .b(\a[16] ), .o1(new_n168));
  nand02aa1d28x5               g073(.a(\b[15] ), .b(\a[16] ), .o1(new_n169));
  norb02aa1n02x5               g074(.a(new_n169), .b(new_n168), .out0(new_n170));
  inv000aa1d42x5               g075(.a(new_n170), .o1(new_n171));
  oaoi13aa1n02x5               g076(.a(new_n171), .b(new_n167), .c(\a[15] ), .d(\b[14] ), .o1(new_n172));
  aoi113aa1n02x5               g077(.a(new_n160), .b(new_n170), .c(new_n165), .d(new_n162), .e(new_n159), .o1(new_n173));
  nor002aa1n02x5               g078(.a(new_n172), .b(new_n173), .o1(\s[16] ));
  nano23aa1n09x5               g079(.a(new_n160), .b(new_n168), .c(new_n169), .d(new_n161), .out0(new_n175));
  nano23aa1n09x5               g080(.a(new_n152), .b(new_n163), .c(new_n159), .d(new_n153), .out0(new_n176));
  nand22aa1n03x5               g081(.a(new_n176), .b(new_n175), .o1(new_n177));
  nano32aa1n03x7               g082(.a(new_n177), .b(new_n145), .c(new_n126), .d(new_n98), .out0(new_n178));
  aoai13aa1n09x5               g083(.a(new_n178), .b(new_n144), .c(new_n107), .d(new_n118), .o1(new_n179));
  inv000aa1n02x5               g084(.a(new_n177), .o1(new_n180));
  nona23aa1n03x5               g085(.a(new_n169), .b(new_n161), .c(new_n160), .d(new_n168), .out0(new_n181));
  tech160nm_fioai012aa1n03p5x5 g086(.a(new_n159), .b(new_n163), .c(new_n152), .o1(new_n182));
  aoi012aa1n02x7               g087(.a(new_n168), .b(new_n160), .c(new_n169), .o1(new_n183));
  tech160nm_fioai012aa1n04x5   g088(.a(new_n183), .b(new_n181), .c(new_n182), .o1(new_n184));
  oaoi13aa1n12x5               g089(.a(new_n184), .b(new_n180), .c(new_n149), .d(new_n150), .o1(new_n185));
  xorc02aa1n12x5               g090(.a(\a[17] ), .b(\b[16] ), .out0(new_n186));
  xnbna2aa1n03x5               g091(.a(new_n186), .b(new_n179), .c(new_n185), .out0(\s[17] ));
  orn002aa1n02x5               g092(.a(\a[17] ), .b(\b[16] ), .o(new_n188));
  nand02aa1d04x5               g093(.a(new_n119), .b(new_n124), .o1(new_n189));
  oabi12aa1n06x5               g094(.a(new_n184), .b(new_n151), .c(new_n177), .out0(new_n190));
  aoai13aa1n02x5               g095(.a(new_n186), .b(new_n190), .c(new_n189), .d(new_n178), .o1(new_n191));
  xnrc02aa1n02x5               g096(.a(\b[17] ), .b(\a[18] ), .out0(new_n192));
  xobna2aa1n03x5               g097(.a(new_n192), .b(new_n191), .c(new_n188), .out0(\s[18] ));
  and002aa1n02x5               g098(.a(\b[17] ), .b(\a[18] ), .o(new_n194));
  nor002aa1d32x5               g099(.a(\b[18] ), .b(\a[19] ), .o1(new_n195));
  nand42aa1n16x5               g100(.a(\b[18] ), .b(\a[19] ), .o1(new_n196));
  norb02aa1n02x5               g101(.a(new_n196), .b(new_n195), .out0(new_n197));
  inv000aa1d42x5               g102(.a(new_n197), .o1(new_n198));
  inv000aa1d42x5               g103(.a(new_n186), .o1(new_n199));
  oai022aa1d24x5               g104(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n200));
  inv000aa1d42x5               g105(.a(new_n200), .o1(new_n201));
  aoai13aa1n06x5               g106(.a(new_n201), .b(new_n199), .c(new_n179), .d(new_n185), .o1(new_n202));
  nona22aa1n03x5               g107(.a(new_n202), .b(new_n198), .c(new_n194), .out0(new_n203));
  aoib12aa1n02x5               g108(.a(new_n197), .b(new_n202), .c(new_n194), .out0(new_n204));
  norb02aa1n02x5               g109(.a(new_n203), .b(new_n204), .out0(\s[19] ));
  xnrc02aa1n02x5               g110(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  inv000aa1n06x5               g111(.a(new_n195), .o1(new_n207));
  nor022aa1n06x5               g112(.a(\b[19] ), .b(\a[20] ), .o1(new_n208));
  nand02aa1n08x5               g113(.a(\b[19] ), .b(\a[20] ), .o1(new_n209));
  norb02aa1n02x5               g114(.a(new_n209), .b(new_n208), .out0(new_n210));
  aobi12aa1n06x5               g115(.a(new_n210), .b(new_n203), .c(new_n207), .out0(new_n211));
  nona22aa1n06x5               g116(.a(new_n203), .b(new_n210), .c(new_n195), .out0(new_n212));
  norb02aa1n03x4               g117(.a(new_n212), .b(new_n211), .out0(\s[20] ));
  nano23aa1n06x5               g118(.a(new_n195), .b(new_n208), .c(new_n209), .d(new_n196), .out0(new_n214));
  nanb03aa1n12x5               g119(.a(new_n192), .b(new_n214), .c(new_n186), .out0(new_n215));
  nona23aa1d16x5               g120(.a(new_n209), .b(new_n196), .c(new_n195), .d(new_n208), .out0(new_n216));
  aob012aa1d18x5               g121(.a(new_n200), .b(\b[17] ), .c(\a[18] ), .out0(new_n217));
  tech160nm_fioaoi03aa1n03p5x5 g122(.a(\a[20] ), .b(\b[19] ), .c(new_n207), .o1(new_n218));
  inv040aa1n06x5               g123(.a(new_n218), .o1(new_n219));
  oai012aa1d24x5               g124(.a(new_n219), .b(new_n216), .c(new_n217), .o1(new_n220));
  inv000aa1d42x5               g125(.a(new_n220), .o1(new_n221));
  aoai13aa1n04x5               g126(.a(new_n221), .b(new_n215), .c(new_n179), .d(new_n185), .o1(new_n222));
  xorb03aa1n02x5               g127(.a(new_n222), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor002aa1n02x5               g128(.a(\b[20] ), .b(\a[21] ), .o1(new_n224));
  xnrc02aa1n12x5               g129(.a(\b[20] ), .b(\a[21] ), .out0(new_n225));
  inv000aa1d42x5               g130(.a(new_n225), .o1(new_n226));
  xnrc02aa1n12x5               g131(.a(\b[21] ), .b(\a[22] ), .out0(new_n227));
  inv000aa1d42x5               g132(.a(new_n227), .o1(new_n228));
  aoai13aa1n02x5               g133(.a(new_n228), .b(new_n224), .c(new_n222), .d(new_n226), .o1(new_n229));
  aoi112aa1n02x5               g134(.a(new_n224), .b(new_n228), .c(new_n222), .d(new_n226), .o1(new_n230));
  norb02aa1n02x5               g135(.a(new_n229), .b(new_n230), .out0(\s[22] ));
  tech160nm_finand02aa1n05x5   g136(.a(new_n179), .b(new_n185), .o1(new_n232));
  nona32aa1n03x5               g137(.a(new_n232), .b(new_n227), .c(new_n225), .d(new_n215), .out0(new_n233));
  oaoi03aa1n02x5               g138(.a(\a[18] ), .b(\b[17] ), .c(new_n188), .o1(new_n234));
  nor042aa1n04x5               g139(.a(new_n227), .b(new_n225), .o1(new_n235));
  aoai13aa1n06x5               g140(.a(new_n235), .b(new_n218), .c(new_n214), .d(new_n234), .o1(new_n236));
  inv000aa1d42x5               g141(.a(\a[22] ), .o1(new_n237));
  inv000aa1d42x5               g142(.a(\b[21] ), .o1(new_n238));
  oao003aa1n12x5               g143(.a(new_n237), .b(new_n238), .c(new_n224), .carry(new_n239));
  inv020aa1n02x5               g144(.a(new_n239), .o1(new_n240));
  nand22aa1n03x5               g145(.a(new_n236), .b(new_n240), .o1(new_n241));
  inv000aa1d42x5               g146(.a(new_n241), .o1(new_n242));
  xnrc02aa1n12x5               g147(.a(\b[22] ), .b(\a[23] ), .out0(new_n243));
  inv000aa1d42x5               g148(.a(new_n243), .o1(new_n244));
  xnbna2aa1n03x5               g149(.a(new_n244), .b(new_n233), .c(new_n242), .out0(\s[23] ));
  inv000aa1d42x5               g150(.a(\a[23] ), .o1(new_n246));
  inv000aa1d42x5               g151(.a(\b[22] ), .o1(new_n247));
  nanp02aa1n02x5               g152(.a(new_n247), .b(new_n246), .o1(new_n248));
  tech160nm_fiaoi012aa1n02p5x5 g153(.a(new_n215), .b(new_n179), .c(new_n185), .o1(new_n249));
  aoai13aa1n02x5               g154(.a(new_n244), .b(new_n241), .c(new_n249), .d(new_n235), .o1(new_n250));
  xnrc02aa1n02x5               g155(.a(\b[23] ), .b(\a[24] ), .out0(new_n251));
  tech160nm_fiaoi012aa1n02p5x5 g156(.a(new_n251), .b(new_n250), .c(new_n248), .o1(new_n252));
  aoi012aa1n06x5               g157(.a(new_n243), .b(new_n233), .c(new_n242), .o1(new_n253));
  nano22aa1n02x4               g158(.a(new_n253), .b(new_n248), .c(new_n251), .out0(new_n254));
  norp02aa1n02x5               g159(.a(new_n252), .b(new_n254), .o1(\s[24] ));
  nona22aa1n06x5               g160(.a(new_n235), .b(new_n243), .c(new_n251), .out0(new_n256));
  nona22aa1n03x5               g161(.a(new_n232), .b(new_n215), .c(new_n256), .out0(new_n257));
  inv000aa1d42x5               g162(.a(\b[23] ), .o1(new_n258));
  aboi22aa1n03x5               g163(.a(\a[24] ), .b(new_n258), .c(new_n246), .d(new_n247), .out0(new_n259));
  and002aa1n02x5               g164(.a(\b[22] ), .b(\a[23] ), .o(new_n260));
  inv000aa1d42x5               g165(.a(new_n260), .o1(new_n261));
  aoai13aa1n09x5               g166(.a(new_n261), .b(new_n239), .c(new_n220), .d(new_n235), .o1(new_n262));
  aoi022aa1n06x5               g167(.a(new_n262), .b(new_n259), .c(\a[24] ), .d(\b[23] ), .o1(new_n263));
  inv000aa1n06x5               g168(.a(new_n263), .o1(new_n264));
  xorc02aa1n12x5               g169(.a(\a[25] ), .b(\b[24] ), .out0(new_n265));
  xnbna2aa1n03x5               g170(.a(new_n265), .b(new_n264), .c(new_n257), .out0(\s[25] ));
  nor042aa1n03x5               g171(.a(\b[24] ), .b(\a[25] ), .o1(new_n267));
  inv000aa1d42x5               g172(.a(new_n267), .o1(new_n268));
  aoi112aa1n02x7               g173(.a(new_n256), .b(new_n215), .c(new_n179), .d(new_n185), .o1(new_n269));
  oai012aa1n03x5               g174(.a(new_n265), .b(new_n269), .c(new_n263), .o1(new_n270));
  xnrc02aa1n12x5               g175(.a(\b[25] ), .b(\a[26] ), .out0(new_n271));
  aoi012aa1n03x5               g176(.a(new_n271), .b(new_n270), .c(new_n268), .o1(new_n272));
  aobi12aa1n06x5               g177(.a(new_n265), .b(new_n264), .c(new_n257), .out0(new_n273));
  nano22aa1n02x4               g178(.a(new_n273), .b(new_n268), .c(new_n271), .out0(new_n274));
  norp02aa1n03x5               g179(.a(new_n272), .b(new_n274), .o1(\s[26] ));
  xorc02aa1n12x5               g180(.a(\a[27] ), .b(\b[26] ), .out0(new_n276));
  inv000aa1d42x5               g181(.a(new_n276), .o1(new_n277));
  and002aa1n02x5               g182(.a(\b[23] ), .b(\a[24] ), .o(new_n278));
  aoai13aa1n03x5               g183(.a(new_n259), .b(new_n260), .c(new_n236), .d(new_n240), .o1(new_n279));
  norb02aa1n02x5               g184(.a(new_n265), .b(new_n271), .out0(new_n280));
  inv000aa1n02x5               g185(.a(new_n280), .o1(new_n281));
  nona22aa1n06x5               g186(.a(new_n279), .b(new_n281), .c(new_n278), .out0(new_n282));
  nor043aa1n03x5               g187(.a(new_n281), .b(new_n256), .c(new_n215), .o1(new_n283));
  aoai13aa1n06x5               g188(.a(new_n283), .b(new_n190), .c(new_n189), .d(new_n178), .o1(new_n284));
  oao003aa1n02x5               g189(.a(\a[26] ), .b(\b[25] ), .c(new_n268), .carry(new_n285));
  aoi013aa1n03x5               g190(.a(new_n277), .b(new_n284), .c(new_n282), .d(new_n285), .o1(new_n286));
  inv030aa1n02x5               g191(.a(new_n283), .o1(new_n287));
  aoai13aa1n06x5               g192(.a(new_n285), .b(new_n287), .c(new_n179), .d(new_n185), .o1(new_n288));
  aoi112aa1n02x5               g193(.a(new_n288), .b(new_n276), .c(new_n263), .d(new_n280), .o1(new_n289));
  norp02aa1n02x5               g194(.a(new_n286), .b(new_n289), .o1(\s[27] ));
  nor042aa1n03x5               g195(.a(\b[26] ), .b(\a[27] ), .o1(new_n291));
  inv000aa1d42x5               g196(.a(new_n291), .o1(new_n292));
  aoi112aa1n06x5               g197(.a(new_n281), .b(new_n278), .c(new_n262), .d(new_n259), .o1(new_n293));
  oai012aa1n03x5               g198(.a(new_n276), .b(new_n288), .c(new_n293), .o1(new_n294));
  xnrc02aa1n12x5               g199(.a(\b[27] ), .b(\a[28] ), .out0(new_n295));
  tech160nm_fiaoi012aa1n05x5   g200(.a(new_n295), .b(new_n294), .c(new_n292), .o1(new_n296));
  nano22aa1n02x4               g201(.a(new_n286), .b(new_n292), .c(new_n295), .out0(new_n297));
  nor002aa1n02x5               g202(.a(new_n296), .b(new_n297), .o1(\s[28] ));
  xnrc02aa1n02x5               g203(.a(\b[28] ), .b(\a[29] ), .out0(new_n299));
  norb02aa1n15x5               g204(.a(new_n276), .b(new_n295), .out0(new_n300));
  oai012aa1n03x5               g205(.a(new_n300), .b(new_n288), .c(new_n293), .o1(new_n301));
  oao003aa1n02x5               g206(.a(\a[28] ), .b(\b[27] ), .c(new_n292), .carry(new_n302));
  aoi012aa1n03x5               g207(.a(new_n299), .b(new_n301), .c(new_n302), .o1(new_n303));
  inv000aa1d42x5               g208(.a(new_n300), .o1(new_n304));
  aoi013aa1n02x4               g209(.a(new_n304), .b(new_n284), .c(new_n282), .d(new_n285), .o1(new_n305));
  nano22aa1n02x4               g210(.a(new_n305), .b(new_n299), .c(new_n302), .out0(new_n306));
  nor002aa1n02x5               g211(.a(new_n303), .b(new_n306), .o1(\s[29] ));
  xorb03aa1n02x5               g212(.a(new_n99), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  xnrc02aa1n02x5               g213(.a(\b[29] ), .b(\a[30] ), .out0(new_n309));
  norb03aa1n12x5               g214(.a(new_n276), .b(new_n299), .c(new_n295), .out0(new_n310));
  oai012aa1n03x5               g215(.a(new_n310), .b(new_n288), .c(new_n293), .o1(new_n311));
  oao003aa1n02x5               g216(.a(\a[29] ), .b(\b[28] ), .c(new_n302), .carry(new_n312));
  aoi012aa1n03x5               g217(.a(new_n309), .b(new_n311), .c(new_n312), .o1(new_n313));
  inv000aa1d42x5               g218(.a(new_n310), .o1(new_n314));
  aoi013aa1n02x4               g219(.a(new_n314), .b(new_n284), .c(new_n282), .d(new_n285), .o1(new_n315));
  nano22aa1n02x4               g220(.a(new_n315), .b(new_n309), .c(new_n312), .out0(new_n316));
  nor002aa1n02x5               g221(.a(new_n313), .b(new_n316), .o1(\s[30] ));
  norb02aa1n02x5               g222(.a(new_n310), .b(new_n309), .out0(new_n318));
  inv000aa1n02x5               g223(.a(new_n318), .o1(new_n319));
  aoi013aa1n02x4               g224(.a(new_n319), .b(new_n284), .c(new_n282), .d(new_n285), .o1(new_n320));
  oao003aa1n02x5               g225(.a(\a[30] ), .b(\b[29] ), .c(new_n312), .carry(new_n321));
  xnrc02aa1n02x5               g226(.a(\b[30] ), .b(\a[31] ), .out0(new_n322));
  nano22aa1n02x5               g227(.a(new_n320), .b(new_n321), .c(new_n322), .out0(new_n323));
  oai012aa1n03x5               g228(.a(new_n318), .b(new_n288), .c(new_n293), .o1(new_n324));
  aoi012aa1n03x5               g229(.a(new_n322), .b(new_n324), .c(new_n321), .o1(new_n325));
  norp02aa1n03x5               g230(.a(new_n325), .b(new_n323), .o1(\s[31] ));
  nona22aa1n02x4               g231(.a(new_n100), .b(new_n101), .c(new_n99), .out0(new_n327));
  xobna2aa1n03x5               g232(.a(new_n104), .b(new_n327), .c(new_n100), .out0(\s[3] ));
  oai012aa1n02x5               g233(.a(new_n105), .b(\b[2] ), .c(\a[3] ), .o1(new_n329));
  xorb03aa1n02x5               g234(.a(new_n329), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  norb02aa1n02x5               g235(.a(new_n111), .b(new_n110), .out0(new_n331));
  xobna2aa1n03x5               g236(.a(new_n331), .b(new_n107), .c(new_n116), .out0(\s[5] ));
  norb02aa1n02x5               g237(.a(new_n115), .b(new_n114), .out0(new_n333));
  nanp03aa1n02x5               g238(.a(new_n107), .b(new_n331), .c(new_n116), .o1(new_n334));
  oaoi13aa1n02x5               g239(.a(new_n333), .b(new_n334), .c(\a[5] ), .d(\b[4] ), .o1(new_n335));
  oai112aa1n02x5               g240(.a(new_n334), .b(new_n333), .c(\b[4] ), .d(\a[5] ), .o1(new_n336));
  nanb02aa1n02x5               g241(.a(new_n335), .b(new_n336), .out0(\s[6] ));
  xnbna2aa1n03x5               g242(.a(new_n113), .b(new_n336), .c(new_n115), .out0(\s[7] ));
  norb02aa1n02x5               g243(.a(new_n108), .b(new_n109), .out0(new_n339));
  nanb03aa1n02x5               g244(.a(new_n113), .b(new_n336), .c(new_n115), .out0(new_n340));
  xnbna2aa1n03x5               g245(.a(new_n339), .b(new_n340), .c(new_n120), .out0(\s[8] ));
  xnbna2aa1n03x5               g246(.a(new_n98), .b(new_n119), .c(new_n124), .out0(\s[9] ));
endmodule


