// Benchmark "adder" written by ABC on Wed Jul 17 15:10:48 2024

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
    new_n132, new_n133, new_n135, new_n136, new_n138, new_n139, new_n140,
    new_n141, new_n142, new_n143, new_n144, new_n145, new_n146, new_n147,
    new_n148, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n160, new_n161, new_n162,
    new_n163, new_n165, new_n166, new_n167, new_n169, new_n170, new_n171,
    new_n172, new_n173, new_n174, new_n175, new_n176, new_n177, new_n179,
    new_n180, new_n181, new_n182, new_n183, new_n184, new_n185, new_n186,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n195, new_n196, new_n197, new_n199, new_n200, new_n201, new_n202,
    new_n203, new_n205, new_n206, new_n207, new_n208, new_n209, new_n212,
    new_n213, new_n214, new_n215, new_n216, new_n217, new_n218, new_n220,
    new_n221, new_n222, new_n223, new_n224, new_n225, new_n226, new_n228,
    new_n229, new_n230, new_n231, new_n232, new_n233, new_n234, new_n236,
    new_n237, new_n238, new_n239, new_n240, new_n241, new_n243, new_n244,
    new_n245, new_n246, new_n247, new_n248, new_n249, new_n250, new_n252,
    new_n253, new_n254, new_n255, new_n256, new_n257, new_n258, new_n260,
    new_n261, new_n262, new_n263, new_n264, new_n266, new_n267, new_n268,
    new_n269, new_n270, new_n271, new_n272, new_n273, new_n274, new_n275,
    new_n276, new_n277, new_n278, new_n279, new_n280, new_n281, new_n283,
    new_n284, new_n285, new_n286, new_n287, new_n288, new_n289, new_n290,
    new_n292, new_n293, new_n294, new_n295, new_n296, new_n297, new_n298,
    new_n301, new_n302, new_n303, new_n304, new_n305, new_n306, new_n307,
    new_n309, new_n310, new_n311, new_n312, new_n313, new_n314, new_n315,
    new_n318, new_n321, new_n322, new_n323, new_n324, new_n325, new_n328;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor042aa1d18x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  inv000aa1d42x5               g002(.a(new_n97), .o1(new_n98));
  nanp02aa1n04x5               g003(.a(\b[7] ), .b(\a[8] ), .o1(new_n99));
  orn002aa1n24x5               g004(.a(\a[8] ), .b(\b[7] ), .o(new_n100));
  nanp02aa1n12x5               g005(.a(new_n100), .b(new_n99), .o1(new_n101));
  inv000aa1d42x5               g006(.a(\a[7] ), .o1(new_n102));
  nanb02aa1n12x5               g007(.a(\b[6] ), .b(new_n102), .out0(new_n103));
  tech160nm_fioaoi03aa1n03p5x5 g008(.a(\a[8] ), .b(\b[7] ), .c(new_n103), .o1(new_n104));
  inv000aa1n02x5               g009(.a(new_n104), .o1(new_n105));
  xnrc02aa1n12x5               g010(.a(\b[6] ), .b(\a[7] ), .out0(new_n106));
  inv040aa1d32x5               g011(.a(\a[6] ), .o1(new_n107));
  inv000aa1d42x5               g012(.a(\b[5] ), .o1(new_n108));
  nor002aa1n03x5               g013(.a(\b[4] ), .b(\a[5] ), .o1(new_n109));
  oaoi03aa1n02x5               g014(.a(new_n107), .b(new_n108), .c(new_n109), .o1(new_n110));
  oai013aa1n06x5               g015(.a(new_n105), .b(new_n110), .c(new_n106), .d(new_n101), .o1(new_n111));
  norp02aa1n02x5               g016(.a(\b[1] ), .b(\a[2] ), .o1(new_n112));
  nand22aa1n03x5               g017(.a(\b[0] ), .b(\a[1] ), .o1(new_n113));
  nanp02aa1n02x5               g018(.a(\b[1] ), .b(\a[2] ), .o1(new_n114));
  tech160nm_fiaoi012aa1n04x5   g019(.a(new_n112), .b(new_n113), .c(new_n114), .o1(new_n115));
  nanp02aa1n03x5               g020(.a(\b[3] ), .b(\a[4] ), .o1(new_n116));
  nor002aa1n02x5               g021(.a(\b[3] ), .b(\a[4] ), .o1(new_n117));
  nor002aa1n02x5               g022(.a(\b[2] ), .b(\a[3] ), .o1(new_n118));
  nand42aa1n02x5               g023(.a(\b[2] ), .b(\a[3] ), .o1(new_n119));
  nona23aa1n09x5               g024(.a(new_n116), .b(new_n119), .c(new_n118), .d(new_n117), .out0(new_n120));
  aoi012aa1n02x5               g025(.a(new_n117), .b(new_n118), .c(new_n116), .o1(new_n121));
  oai012aa1n06x5               g026(.a(new_n121), .b(new_n120), .c(new_n115), .o1(new_n122));
  oai112aa1n03x5               g027(.a(new_n100), .b(new_n99), .c(\b[6] ), .d(\a[7] ), .o1(new_n123));
  nanp02aa1n02x5               g028(.a(\b[5] ), .b(\a[6] ), .o1(new_n124));
  xnrc02aa1n02x5               g029(.a(\b[4] ), .b(\a[5] ), .out0(new_n125));
  aoi022aa1n02x5               g030(.a(new_n108), .b(new_n107), .c(\a[7] ), .d(\b[6] ), .o1(new_n126));
  nano23aa1n06x5               g031(.a(new_n123), .b(new_n125), .c(new_n126), .d(new_n124), .out0(new_n127));
  and002aa1n02x5               g032(.a(\b[8] ), .b(\a[9] ), .o(new_n128));
  norp02aa1n02x5               g033(.a(new_n128), .b(new_n97), .o1(new_n129));
  aoai13aa1n06x5               g034(.a(new_n129), .b(new_n111), .c(new_n122), .d(new_n127), .o1(new_n130));
  nor002aa1n16x5               g035(.a(\b[9] ), .b(\a[10] ), .o1(new_n131));
  and002aa1n12x5               g036(.a(\b[9] ), .b(\a[10] ), .o(new_n132));
  norp02aa1n02x5               g037(.a(new_n132), .b(new_n131), .o1(new_n133));
  xnbna2aa1n03x5               g038(.a(new_n133), .b(new_n130), .c(new_n98), .out0(\s[10] ));
  inv000aa1d42x5               g039(.a(new_n131), .o1(new_n135));
  aoi013aa1n02x4               g040(.a(new_n132), .b(new_n130), .c(new_n135), .d(new_n98), .o1(new_n136));
  xorb03aa1n02x5               g041(.a(new_n136), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  nor042aa1d18x5               g042(.a(\b[10] ), .b(\a[11] ), .o1(new_n138));
  inv000aa1d42x5               g043(.a(new_n138), .o1(new_n139));
  nand42aa1n06x5               g044(.a(\b[10] ), .b(\a[11] ), .o1(new_n140));
  norb02aa1n02x5               g045(.a(new_n140), .b(new_n138), .out0(new_n141));
  inv000aa1n02x5               g046(.a(new_n141), .o1(new_n142));
  aoi113aa1n02x5               g047(.a(new_n142), .b(new_n132), .c(new_n130), .d(new_n135), .e(new_n98), .o1(new_n143));
  nor042aa1n06x5               g048(.a(\b[11] ), .b(\a[12] ), .o1(new_n144));
  nand42aa1n10x5               g049(.a(\b[11] ), .b(\a[12] ), .o1(new_n145));
  nanb02aa1n02x5               g050(.a(new_n144), .b(new_n145), .out0(new_n146));
  nano22aa1n02x4               g051(.a(new_n143), .b(new_n139), .c(new_n146), .out0(new_n147));
  oab012aa1n02x4               g052(.a(new_n146), .b(new_n143), .c(new_n138), .out0(new_n148));
  norp02aa1n02x5               g053(.a(new_n148), .b(new_n147), .o1(\s[12] ));
  inv000aa1d42x5               g054(.a(new_n101), .o1(new_n150));
  inv000aa1d42x5               g055(.a(new_n106), .o1(new_n151));
  oao003aa1n02x5               g056(.a(new_n107), .b(new_n108), .c(new_n109), .carry(new_n152));
  aoi013aa1n06x4               g057(.a(new_n104), .b(new_n152), .c(new_n151), .d(new_n150), .o1(new_n153));
  nanp02aa1n02x5               g058(.a(new_n122), .b(new_n127), .o1(new_n154));
  nor043aa1d12x5               g059(.a(new_n132), .b(new_n131), .c(new_n97), .o1(new_n155));
  nano22aa1d15x5               g060(.a(new_n144), .b(new_n140), .c(new_n145), .out0(new_n156));
  nona23aa1d16x5               g061(.a(new_n156), .b(new_n155), .c(new_n128), .d(new_n138), .out0(new_n157));
  oabi12aa1n18x5               g062(.a(new_n132), .b(new_n97), .c(new_n131), .out0(new_n158));
  tech160nm_fioai012aa1n03p5x5 g063(.a(new_n145), .b(new_n144), .c(new_n138), .o1(new_n159));
  nona23aa1n09x5               g064(.a(new_n145), .b(new_n140), .c(new_n138), .d(new_n144), .out0(new_n160));
  oai012aa1d24x5               g065(.a(new_n159), .b(new_n160), .c(new_n158), .o1(new_n161));
  inv000aa1d42x5               g066(.a(new_n161), .o1(new_n162));
  aoai13aa1n02x5               g067(.a(new_n162), .b(new_n157), .c(new_n154), .d(new_n153), .o1(new_n163));
  xorb03aa1n02x5               g068(.a(new_n163), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  inv000aa1d42x5               g069(.a(\a[13] ), .o1(new_n165));
  inv000aa1d42x5               g070(.a(\b[12] ), .o1(new_n166));
  oaoi03aa1n02x5               g071(.a(new_n165), .b(new_n166), .c(new_n163), .o1(new_n167));
  xnrb03aa1n03x5               g072(.a(new_n167), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  inv000aa1d42x5               g073(.a(new_n157), .o1(new_n169));
  aoai13aa1n03x5               g074(.a(new_n169), .b(new_n111), .c(new_n122), .d(new_n127), .o1(new_n170));
  nor022aa1n04x5               g075(.a(\b[13] ), .b(\a[14] ), .o1(new_n171));
  nand42aa1n03x5               g076(.a(\b[13] ), .b(\a[14] ), .o1(new_n172));
  aoai13aa1n06x5               g077(.a(new_n172), .b(new_n171), .c(new_n165), .d(new_n166), .o1(new_n173));
  norp02aa1n02x5               g078(.a(\b[12] ), .b(\a[13] ), .o1(new_n174));
  nand42aa1n03x5               g079(.a(\b[12] ), .b(\a[13] ), .o1(new_n175));
  nona23aa1n09x5               g080(.a(new_n172), .b(new_n175), .c(new_n174), .d(new_n171), .out0(new_n176));
  aoai13aa1n04x5               g081(.a(new_n173), .b(new_n176), .c(new_n170), .d(new_n162), .o1(new_n177));
  xorb03aa1n02x5               g082(.a(new_n177), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  norp02aa1n03x5               g083(.a(\b[14] ), .b(\a[15] ), .o1(new_n179));
  nand42aa1n03x5               g084(.a(\b[14] ), .b(\a[15] ), .o1(new_n180));
  nor022aa1n03x5               g085(.a(\b[15] ), .b(\a[16] ), .o1(new_n181));
  nand42aa1n03x5               g086(.a(\b[15] ), .b(\a[16] ), .o1(new_n182));
  nanb02aa1n02x5               g087(.a(new_n181), .b(new_n182), .out0(new_n183));
  inv000aa1d42x5               g088(.a(new_n183), .o1(new_n184));
  aoi112aa1n02x5               g089(.a(new_n179), .b(new_n184), .c(new_n177), .d(new_n180), .o1(new_n185));
  aoai13aa1n02x5               g090(.a(new_n184), .b(new_n179), .c(new_n177), .d(new_n180), .o1(new_n186));
  norb02aa1n03x4               g091(.a(new_n186), .b(new_n185), .out0(\s[16] ));
  nano23aa1n03x7               g092(.a(new_n174), .b(new_n171), .c(new_n172), .d(new_n175), .out0(new_n188));
  nano23aa1n02x5               g093(.a(new_n179), .b(new_n181), .c(new_n182), .d(new_n180), .out0(new_n189));
  nano22aa1n12x5               g094(.a(new_n157), .b(new_n188), .c(new_n189), .out0(new_n190));
  aoai13aa1n12x5               g095(.a(new_n190), .b(new_n111), .c(new_n127), .d(new_n122), .o1(new_n191));
  nona23aa1n03x5               g096(.a(new_n182), .b(new_n180), .c(new_n179), .d(new_n181), .out0(new_n192));
  norp02aa1n03x5               g097(.a(new_n192), .b(new_n176), .o1(new_n193));
  oa0012aa1n02x5               g098(.a(new_n182), .b(new_n181), .c(new_n179), .o(new_n194));
  norp02aa1n02x5               g099(.a(new_n192), .b(new_n173), .o1(new_n195));
  aoi112aa1n09x5               g100(.a(new_n195), .b(new_n194), .c(new_n161), .d(new_n193), .o1(new_n196));
  nanp02aa1n06x5               g101(.a(new_n191), .b(new_n196), .o1(new_n197));
  xorb03aa1n02x5               g102(.a(new_n197), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv000aa1d42x5               g103(.a(\a[17] ), .o1(new_n199));
  inv000aa1d42x5               g104(.a(\b[16] ), .o1(new_n200));
  nanp02aa1n02x5               g105(.a(new_n200), .b(new_n199), .o1(new_n201));
  and002aa1n02x5               g106(.a(\b[16] ), .b(\a[17] ), .o(new_n202));
  aoai13aa1n02x5               g107(.a(new_n201), .b(new_n202), .c(new_n191), .d(new_n196), .o1(new_n203));
  xorb03aa1n02x5               g108(.a(new_n203), .b(\b[17] ), .c(\a[18] ), .out0(\s[18] ));
  norp02aa1n04x5               g109(.a(\b[17] ), .b(\a[18] ), .o1(new_n205));
  nanp02aa1n02x5               g110(.a(\b[17] ), .b(\a[18] ), .o1(new_n206));
  nona23aa1n06x5               g111(.a(new_n206), .b(new_n201), .c(new_n202), .d(new_n205), .out0(new_n207));
  aoai13aa1n06x5               g112(.a(new_n206), .b(new_n205), .c(new_n199), .d(new_n200), .o1(new_n208));
  aoai13aa1n06x5               g113(.a(new_n208), .b(new_n207), .c(new_n191), .d(new_n196), .o1(new_n209));
  xorb03aa1n02x5               g114(.a(new_n209), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g115(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor022aa1n16x5               g116(.a(\b[18] ), .b(\a[19] ), .o1(new_n212));
  nanp02aa1n04x5               g117(.a(\b[18] ), .b(\a[19] ), .o1(new_n213));
  nor022aa1n12x5               g118(.a(\b[19] ), .b(\a[20] ), .o1(new_n214));
  nand02aa1n04x5               g119(.a(\b[19] ), .b(\a[20] ), .o1(new_n215));
  norb02aa1n02x5               g120(.a(new_n215), .b(new_n214), .out0(new_n216));
  aoi112aa1n02x7               g121(.a(new_n212), .b(new_n216), .c(new_n209), .d(new_n213), .o1(new_n217));
  aoai13aa1n03x5               g122(.a(new_n216), .b(new_n212), .c(new_n209), .d(new_n213), .o1(new_n218));
  norb02aa1n03x4               g123(.a(new_n218), .b(new_n217), .out0(\s[20] ));
  nona23aa1d18x5               g124(.a(new_n215), .b(new_n213), .c(new_n212), .d(new_n214), .out0(new_n220));
  inv000aa1d42x5               g125(.a(new_n220), .o1(new_n221));
  nanb02aa1n02x5               g126(.a(new_n207), .b(new_n221), .out0(new_n222));
  tech160nm_fioai012aa1n05x5   g127(.a(new_n215), .b(new_n214), .c(new_n212), .o1(new_n223));
  oai012aa1d24x5               g128(.a(new_n223), .b(new_n220), .c(new_n208), .o1(new_n224));
  inv000aa1d42x5               g129(.a(new_n224), .o1(new_n225));
  aoai13aa1n06x5               g130(.a(new_n225), .b(new_n222), .c(new_n191), .d(new_n196), .o1(new_n226));
  xorb03aa1n02x5               g131(.a(new_n226), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor022aa1n08x5               g132(.a(\b[20] ), .b(\a[21] ), .o1(new_n228));
  nanp02aa1n04x5               g133(.a(\b[20] ), .b(\a[21] ), .o1(new_n229));
  nor022aa1n04x5               g134(.a(\b[21] ), .b(\a[22] ), .o1(new_n230));
  nanp02aa1n04x5               g135(.a(\b[21] ), .b(\a[22] ), .o1(new_n231));
  norb02aa1n02x5               g136(.a(new_n231), .b(new_n230), .out0(new_n232));
  aoi112aa1n02x7               g137(.a(new_n228), .b(new_n232), .c(new_n226), .d(new_n229), .o1(new_n233));
  aoai13aa1n03x5               g138(.a(new_n232), .b(new_n228), .c(new_n226), .d(new_n229), .o1(new_n234));
  norb02aa1n03x4               g139(.a(new_n234), .b(new_n233), .out0(\s[22] ));
  nona23aa1n12x5               g140(.a(new_n231), .b(new_n229), .c(new_n228), .d(new_n230), .out0(new_n236));
  inv000aa1d42x5               g141(.a(new_n236), .o1(new_n237));
  nanb03aa1n02x5               g142(.a(new_n207), .b(new_n237), .c(new_n221), .out0(new_n238));
  oai012aa1n02x5               g143(.a(new_n231), .b(new_n230), .c(new_n228), .o1(new_n239));
  aobi12aa1n02x5               g144(.a(new_n239), .b(new_n224), .c(new_n237), .out0(new_n240));
  aoai13aa1n06x5               g145(.a(new_n240), .b(new_n238), .c(new_n191), .d(new_n196), .o1(new_n241));
  xorb03aa1n02x5               g146(.a(new_n241), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n03x5               g147(.a(\b[22] ), .b(\a[23] ), .o1(new_n243));
  nanp02aa1n02x5               g148(.a(\b[22] ), .b(\a[23] ), .o1(new_n244));
  nor002aa1n02x5               g149(.a(\b[23] ), .b(\a[24] ), .o1(new_n245));
  nanp02aa1n02x5               g150(.a(\b[23] ), .b(\a[24] ), .o1(new_n246));
  nanb02aa1n02x5               g151(.a(new_n245), .b(new_n246), .out0(new_n247));
  inv000aa1d42x5               g152(.a(new_n247), .o1(new_n248));
  aoi112aa1n02x7               g153(.a(new_n243), .b(new_n248), .c(new_n241), .d(new_n244), .o1(new_n249));
  aoai13aa1n03x5               g154(.a(new_n248), .b(new_n243), .c(new_n241), .d(new_n244), .o1(new_n250));
  norb02aa1n03x4               g155(.a(new_n250), .b(new_n249), .out0(\s[24] ));
  nona23aa1n02x4               g156(.a(new_n246), .b(new_n244), .c(new_n243), .d(new_n245), .out0(new_n252));
  nor042aa1n06x5               g157(.a(new_n252), .b(new_n236), .o1(new_n253));
  nona22aa1n03x5               g158(.a(new_n253), .b(new_n207), .c(new_n220), .out0(new_n254));
  norp02aa1n03x5               g159(.a(new_n252), .b(new_n239), .o1(new_n255));
  oa0012aa1n02x5               g160(.a(new_n246), .b(new_n245), .c(new_n243), .o(new_n256));
  aoi112aa1n03x5               g161(.a(new_n256), .b(new_n255), .c(new_n224), .d(new_n253), .o1(new_n257));
  aoai13aa1n06x5               g162(.a(new_n257), .b(new_n254), .c(new_n191), .d(new_n196), .o1(new_n258));
  xorb03aa1n02x5               g163(.a(new_n258), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g164(.a(\b[24] ), .b(\a[25] ), .o1(new_n260));
  nanp02aa1n02x5               g165(.a(\b[24] ), .b(\a[25] ), .o1(new_n261));
  xorc02aa1n12x5               g166(.a(\a[26] ), .b(\b[25] ), .out0(new_n262));
  aoi112aa1n02x7               g167(.a(new_n260), .b(new_n262), .c(new_n258), .d(new_n261), .o1(new_n263));
  aoai13aa1n03x5               g168(.a(new_n262), .b(new_n260), .c(new_n258), .d(new_n261), .o1(new_n264));
  norb02aa1n03x4               g169(.a(new_n264), .b(new_n263), .out0(\s[26] ));
  nanp02aa1n03x5               g170(.a(new_n154), .b(new_n153), .o1(new_n266));
  nanp02aa1n02x5               g171(.a(new_n161), .b(new_n193), .o1(new_n267));
  nona22aa1n02x4               g172(.a(new_n267), .b(new_n195), .c(new_n194), .out0(new_n268));
  nano22aa1n03x7               g173(.a(new_n260), .b(new_n262), .c(new_n261), .out0(new_n269));
  inv000aa1d42x5               g174(.a(new_n269), .o1(new_n270));
  nor042aa1n06x5               g175(.a(new_n254), .b(new_n270), .o1(new_n271));
  aoai13aa1n06x5               g176(.a(new_n271), .b(new_n268), .c(new_n266), .d(new_n190), .o1(new_n272));
  nanp02aa1n02x5               g177(.a(new_n224), .b(new_n253), .o1(new_n273));
  nona22aa1n03x5               g178(.a(new_n273), .b(new_n255), .c(new_n256), .out0(new_n274));
  nanp02aa1n02x5               g179(.a(\b[25] ), .b(\a[26] ), .o1(new_n275));
  inv000aa1d42x5               g180(.a(\a[25] ), .o1(new_n276));
  oaib12aa1n02x5               g181(.a(new_n262), .b(\b[24] ), .c(new_n276), .out0(new_n277));
  aoi022aa1n06x5               g182(.a(new_n274), .b(new_n269), .c(new_n275), .d(new_n277), .o1(new_n278));
  nor042aa1n03x5               g183(.a(\b[26] ), .b(\a[27] ), .o1(new_n279));
  nanp02aa1n02x5               g184(.a(\b[26] ), .b(\a[27] ), .o1(new_n280));
  norb02aa1n02x5               g185(.a(new_n280), .b(new_n279), .out0(new_n281));
  xnbna2aa1n03x5               g186(.a(new_n281), .b(new_n272), .c(new_n278), .out0(\s[27] ));
  inv000aa1d42x5               g187(.a(new_n279), .o1(new_n283));
  aobi12aa1n02x5               g188(.a(new_n281), .b(new_n272), .c(new_n278), .out0(new_n284));
  xnrc02aa1n02x5               g189(.a(\b[27] ), .b(\a[28] ), .out0(new_n285));
  nano22aa1n02x4               g190(.a(new_n284), .b(new_n283), .c(new_n285), .out0(new_n286));
  nanp02aa1n02x5               g191(.a(new_n277), .b(new_n275), .o1(new_n287));
  oai012aa1n04x7               g192(.a(new_n287), .b(new_n257), .c(new_n270), .o1(new_n288));
  aoai13aa1n03x5               g193(.a(new_n281), .b(new_n288), .c(new_n197), .d(new_n271), .o1(new_n289));
  aoi012aa1n03x5               g194(.a(new_n285), .b(new_n289), .c(new_n283), .o1(new_n290));
  norp02aa1n03x5               g195(.a(new_n290), .b(new_n286), .o1(\s[28] ));
  xnrc02aa1n02x5               g196(.a(\b[28] ), .b(\a[29] ), .out0(new_n292));
  nano22aa1n02x4               g197(.a(new_n285), .b(new_n283), .c(new_n280), .out0(new_n293));
  aoai13aa1n03x5               g198(.a(new_n293), .b(new_n288), .c(new_n197), .d(new_n271), .o1(new_n294));
  oao003aa1n02x5               g199(.a(\a[28] ), .b(\b[27] ), .c(new_n283), .carry(new_n295));
  aoi012aa1n03x5               g200(.a(new_n292), .b(new_n294), .c(new_n295), .o1(new_n296));
  aobi12aa1n02x7               g201(.a(new_n293), .b(new_n272), .c(new_n278), .out0(new_n297));
  nano22aa1n02x4               g202(.a(new_n297), .b(new_n292), .c(new_n295), .out0(new_n298));
  norp02aa1n03x5               g203(.a(new_n296), .b(new_n298), .o1(\s[29] ));
  xorb03aa1n02x5               g204(.a(new_n113), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  oao003aa1n02x5               g205(.a(\a[29] ), .b(\b[28] ), .c(new_n295), .carry(new_n301));
  nano23aa1n02x4               g206(.a(new_n292), .b(new_n285), .c(new_n280), .d(new_n283), .out0(new_n302));
  aoai13aa1n03x5               g207(.a(new_n302), .b(new_n288), .c(new_n197), .d(new_n271), .o1(new_n303));
  xnrc02aa1n02x5               g208(.a(\b[29] ), .b(\a[30] ), .out0(new_n304));
  aoi012aa1n03x5               g209(.a(new_n304), .b(new_n303), .c(new_n301), .o1(new_n305));
  aobi12aa1n02x7               g210(.a(new_n302), .b(new_n272), .c(new_n278), .out0(new_n306));
  nano22aa1n02x4               g211(.a(new_n306), .b(new_n301), .c(new_n304), .out0(new_n307));
  norp02aa1n03x5               g212(.a(new_n305), .b(new_n307), .o1(\s[30] ));
  norb03aa1n02x5               g213(.a(new_n293), .b(new_n292), .c(new_n304), .out0(new_n309));
  aobi12aa1n02x7               g214(.a(new_n309), .b(new_n272), .c(new_n278), .out0(new_n310));
  oao003aa1n02x5               g215(.a(\a[30] ), .b(\b[29] ), .c(new_n301), .carry(new_n311));
  xnrc02aa1n02x5               g216(.a(\b[30] ), .b(\a[31] ), .out0(new_n312));
  nano22aa1n02x4               g217(.a(new_n310), .b(new_n311), .c(new_n312), .out0(new_n313));
  aoai13aa1n03x5               g218(.a(new_n309), .b(new_n288), .c(new_n197), .d(new_n271), .o1(new_n314));
  aoi012aa1n03x5               g219(.a(new_n312), .b(new_n314), .c(new_n311), .o1(new_n315));
  norp02aa1n03x5               g220(.a(new_n315), .b(new_n313), .o1(\s[31] ));
  xnrb03aa1n02x5               g221(.a(new_n115), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g222(.a(\a[3] ), .b(\b[2] ), .c(new_n115), .o1(new_n318));
  xorb03aa1n02x5               g223(.a(new_n318), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g224(.a(new_n122), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  oaoi13aa1n02x5               g225(.a(new_n125), .b(new_n121), .c(new_n120), .d(new_n115), .o1(new_n321));
  xorc02aa1n02x5               g226(.a(\a[6] ), .b(\b[5] ), .out0(new_n322));
  inv000aa1d42x5               g227(.a(new_n322), .o1(new_n323));
  oai012aa1n02x5               g228(.a(new_n323), .b(new_n321), .c(new_n109), .o1(new_n324));
  nona22aa1n02x4               g229(.a(new_n322), .b(new_n321), .c(new_n109), .out0(new_n325));
  nanp02aa1n02x5               g230(.a(new_n325), .b(new_n324), .o1(\s[6] ));
  xnbna2aa1n03x5               g231(.a(new_n106), .b(new_n325), .c(new_n124), .out0(\s[7] ));
  nanp03aa1n02x5               g232(.a(new_n325), .b(new_n151), .c(new_n124), .o1(new_n328));
  xnbna2aa1n03x5               g233(.a(new_n150), .b(new_n328), .c(new_n103), .out0(\s[8] ));
  xnbna2aa1n03x5               g234(.a(new_n129), .b(new_n154), .c(new_n153), .out0(\s[9] ));
endmodule


