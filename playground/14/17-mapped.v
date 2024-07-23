// Benchmark "adder" written by ABC on Wed Jul 17 19:15:53 2024

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
    new_n125, new_n126, new_n128, new_n129, new_n130, new_n131, new_n133,
    new_n134, new_n135, new_n136, new_n137, new_n138, new_n139, new_n141,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n150, new_n151, new_n152, new_n153, new_n154, new_n155, new_n156,
    new_n158, new_n159, new_n160, new_n161, new_n162, new_n164, new_n165,
    new_n166, new_n167, new_n168, new_n169, new_n170, new_n171, new_n172,
    new_n173, new_n175, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n189, new_n190, new_n191, new_n193, new_n194, new_n195, new_n196,
    new_n197, new_n198, new_n199, new_n200, new_n201, new_n202, new_n203,
    new_n204, new_n205, new_n206, new_n207, new_n208, new_n209, new_n210,
    new_n211, new_n212, new_n215, new_n216, new_n217, new_n218, new_n219,
    new_n220, new_n221, new_n222, new_n223, new_n224, new_n226, new_n227,
    new_n228, new_n229, new_n230, new_n231, new_n232, new_n233, new_n234,
    new_n235, new_n236, new_n237, new_n238, new_n240, new_n241, new_n242,
    new_n243, new_n244, new_n245, new_n246, new_n247, new_n249, new_n250,
    new_n251, new_n252, new_n253, new_n254, new_n255, new_n256, new_n258,
    new_n259, new_n260, new_n261, new_n262, new_n263, new_n264, new_n265,
    new_n266, new_n267, new_n269, new_n270, new_n271, new_n272, new_n273,
    new_n274, new_n275, new_n276, new_n277, new_n278, new_n279, new_n281,
    new_n282, new_n283, new_n284, new_n285, new_n286, new_n287, new_n289,
    new_n290, new_n291, new_n292, new_n293, new_n294, new_n295, new_n296,
    new_n298, new_n299, new_n300, new_n301, new_n302, new_n303, new_n304,
    new_n305, new_n306, new_n307, new_n308, new_n310, new_n311, new_n312,
    new_n313, new_n314, new_n315, new_n316, new_n318, new_n320, new_n321,
    new_n322, new_n323, new_n324, new_n325, new_n326, new_n328, new_n329,
    new_n330, new_n331, new_n332, new_n333, new_n334, new_n335, new_n338,
    new_n340, new_n341, new_n343, new_n344, new_n345, new_n346, new_n347,
    new_n349, new_n351;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  xnrc02aa1n12x5               g001(.a(\b[9] ), .b(\a[10] ), .out0(new_n97));
  inv040aa1n02x5               g002(.a(new_n97), .o1(new_n98));
  nor002aa1d32x5               g003(.a(\b[8] ), .b(\a[9] ), .o1(new_n99));
  inv000aa1d42x5               g004(.a(new_n99), .o1(new_n100));
  nor002aa1n16x5               g005(.a(\b[2] ), .b(\a[3] ), .o1(new_n101));
  and002aa1n24x5               g006(.a(\b[3] ), .b(\a[4] ), .o(new_n102));
  norp02aa1n02x5               g007(.a(\b[3] ), .b(\a[4] ), .o1(new_n103));
  nor043aa1n06x5               g008(.a(new_n102), .b(new_n103), .c(new_n101), .o1(new_n104));
  nanp02aa1n04x5               g009(.a(\b[2] ), .b(\a[3] ), .o1(new_n105));
  nanb02aa1n06x5               g010(.a(new_n101), .b(new_n105), .out0(new_n106));
  oai112aa1n06x5               g011(.a(\a[1] ), .b(\b[0] ), .c(\b[1] ), .d(\a[2] ), .o1(new_n107));
  nanp02aa1n04x5               g012(.a(\b[1] ), .b(\a[2] ), .o1(new_n108));
  nanp02aa1n09x5               g013(.a(new_n107), .b(new_n108), .o1(new_n109));
  oai012aa1d24x5               g014(.a(new_n104), .b(new_n109), .c(new_n106), .o1(new_n110));
  nor002aa1d24x5               g015(.a(\b[6] ), .b(\a[7] ), .o1(new_n111));
  nor002aa1d32x5               g016(.a(\b[7] ), .b(\a[8] ), .o1(new_n112));
  nand42aa1d28x5               g017(.a(\b[7] ), .b(\a[8] ), .o1(new_n113));
  aoi022aa1d24x5               g018(.a(\b[6] ), .b(\a[7] ), .c(\a[6] ), .d(\b[5] ), .o1(new_n114));
  nona23aa1d16x5               g019(.a(new_n114), .b(new_n113), .c(new_n111), .d(new_n112), .out0(new_n115));
  aoi022aa1d24x5               g020(.a(\b[4] ), .b(\a[5] ), .c(\a[4] ), .d(\b[3] ), .o1(new_n116));
  oai122aa1n03x5               g021(.a(new_n116), .b(\a[6] ), .c(\b[5] ), .d(\a[5] ), .e(\b[4] ), .o1(new_n117));
  nor042aa1n04x5               g022(.a(new_n115), .b(new_n117), .o1(new_n118));
  inv000aa1d42x5               g023(.a(new_n113), .o1(new_n119));
  norb03aa1d15x5               g024(.a(new_n113), .b(new_n111), .c(new_n112), .out0(new_n120));
  oaih22aa1d12x5               g025(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n121));
  aoi012aa1d24x5               g026(.a(new_n121), .b(\a[6] ), .c(\b[5] ), .o1(new_n122));
  oai022aa1d18x5               g027(.a(new_n115), .b(new_n122), .c(new_n120), .d(new_n119), .o1(new_n123));
  xnrc02aa1n12x5               g028(.a(\b[8] ), .b(\a[9] ), .out0(new_n124));
  inv030aa1n04x5               g029(.a(new_n124), .o1(new_n125));
  aoai13aa1n06x5               g030(.a(new_n125), .b(new_n123), .c(new_n110), .d(new_n118), .o1(new_n126));
  xnbna2aa1n03x5               g031(.a(new_n98), .b(new_n126), .c(new_n100), .out0(\s[10] ));
  norp02aa1n24x5               g032(.a(\b[9] ), .b(\a[10] ), .o1(new_n128));
  nand02aa1n06x5               g033(.a(\b[9] ), .b(\a[10] ), .o1(new_n129));
  oai012aa1n18x5               g034(.a(new_n129), .b(new_n99), .c(new_n128), .o1(new_n130));
  aoai13aa1n06x5               g035(.a(new_n130), .b(new_n97), .c(new_n126), .d(new_n100), .o1(new_n131));
  xorb03aa1n02x5               g036(.a(new_n131), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  nor042aa1d18x5               g037(.a(\b[10] ), .b(\a[11] ), .o1(new_n133));
  nand42aa1d28x5               g038(.a(\b[10] ), .b(\a[11] ), .o1(new_n134));
  nor002aa1d32x5               g039(.a(\b[11] ), .b(\a[12] ), .o1(new_n135));
  nand42aa1d28x5               g040(.a(\b[11] ), .b(\a[12] ), .o1(new_n136));
  norb02aa1n02x5               g041(.a(new_n136), .b(new_n135), .out0(new_n137));
  aoai13aa1n03x5               g042(.a(new_n137), .b(new_n133), .c(new_n131), .d(new_n134), .o1(new_n138));
  aoi112aa1n02x7               g043(.a(new_n133), .b(new_n137), .c(new_n131), .d(new_n134), .o1(new_n139));
  norb02aa1n03x4               g044(.a(new_n138), .b(new_n139), .out0(\s[12] ));
  nona23aa1n09x5               g045(.a(new_n136), .b(new_n134), .c(new_n133), .d(new_n135), .out0(new_n141));
  nor003aa1n03x5               g046(.a(new_n141), .b(new_n124), .c(new_n97), .o1(new_n142));
  aoai13aa1n06x5               g047(.a(new_n142), .b(new_n123), .c(new_n110), .d(new_n118), .o1(new_n143));
  inv020aa1n08x5               g048(.a(new_n130), .o1(new_n144));
  nano23aa1n09x5               g049(.a(new_n133), .b(new_n135), .c(new_n136), .d(new_n134), .out0(new_n145));
  tech160nm_fioai012aa1n04x5   g050(.a(new_n136), .b(new_n135), .c(new_n133), .o1(new_n146));
  aobi12aa1n09x5               g051(.a(new_n146), .b(new_n145), .c(new_n144), .out0(new_n147));
  nanp02aa1n06x5               g052(.a(new_n143), .b(new_n147), .o1(new_n148));
  xorb03aa1n02x5               g053(.a(new_n148), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  inv040aa1d32x5               g054(.a(\a[14] ), .o1(new_n150));
  inv000aa1d42x5               g055(.a(\b[13] ), .o1(new_n151));
  nanp02aa1n04x5               g056(.a(new_n151), .b(new_n150), .o1(new_n152));
  nand02aa1d24x5               g057(.a(\b[13] ), .b(\a[14] ), .o1(new_n153));
  norp02aa1n12x5               g058(.a(\b[12] ), .b(\a[13] ), .o1(new_n154));
  nand42aa1d28x5               g059(.a(\b[12] ), .b(\a[13] ), .o1(new_n155));
  tech160nm_fiaoi012aa1n05x5   g060(.a(new_n154), .b(new_n148), .c(new_n155), .o1(new_n156));
  xnbna2aa1n03x5               g061(.a(new_n156), .b(new_n152), .c(new_n153), .out0(\s[14] ));
  nor022aa1n08x5               g062(.a(\b[13] ), .b(\a[14] ), .o1(new_n158));
  oa0012aa1n02x5               g063(.a(new_n153), .b(new_n158), .c(new_n154), .o(new_n159));
  inv000aa1n02x5               g064(.a(new_n159), .o1(new_n160));
  nona23aa1n09x5               g065(.a(new_n153), .b(new_n155), .c(new_n154), .d(new_n158), .out0(new_n161));
  aoai13aa1n06x5               g066(.a(new_n160), .b(new_n161), .c(new_n143), .d(new_n147), .o1(new_n162));
  xorb03aa1n02x5               g067(.a(new_n162), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  inv000aa1d42x5               g068(.a(\a[15] ), .o1(new_n164));
  nanb02aa1d36x5               g069(.a(\b[14] ), .b(new_n164), .out0(new_n165));
  inv000aa1d42x5               g070(.a(new_n165), .o1(new_n166));
  xorc02aa1n12x5               g071(.a(\a[15] ), .b(\b[14] ), .out0(new_n167));
  nor022aa1n06x5               g072(.a(\b[15] ), .b(\a[16] ), .o1(new_n168));
  nand42aa1d28x5               g073(.a(\b[15] ), .b(\a[16] ), .o1(new_n169));
  nanb02aa1n02x5               g074(.a(new_n168), .b(new_n169), .out0(new_n170));
  inv000aa1d42x5               g075(.a(new_n170), .o1(new_n171));
  aoai13aa1n03x5               g076(.a(new_n171), .b(new_n166), .c(new_n162), .d(new_n167), .o1(new_n172));
  aoi112aa1n02x7               g077(.a(new_n166), .b(new_n171), .c(new_n162), .d(new_n167), .o1(new_n173));
  norb02aa1n03x4               g078(.a(new_n172), .b(new_n173), .out0(\s[16] ));
  aoi012aa1d24x5               g079(.a(new_n123), .b(new_n118), .c(new_n110), .o1(new_n175));
  xnrc02aa1n02x5               g080(.a(\b[14] ), .b(\a[15] ), .out0(new_n176));
  nanb03aa1n02x5               g081(.a(new_n168), .b(new_n169), .c(new_n153), .out0(new_n177));
  nor043aa1n02x5               g082(.a(new_n161), .b(new_n176), .c(new_n177), .o1(new_n178));
  nand02aa1d04x5               g083(.a(new_n178), .b(new_n142), .o1(new_n179));
  oai012aa1n02x7               g084(.a(new_n146), .b(new_n141), .c(new_n130), .o1(new_n180));
  oai112aa1n03x5               g085(.a(new_n152), .b(new_n153), .c(\b[12] ), .d(\a[13] ), .o1(new_n181));
  nano22aa1n03x7               g086(.a(new_n168), .b(new_n153), .c(new_n169), .out0(new_n182));
  nanp03aa1n03x5               g087(.a(new_n167), .b(new_n182), .c(new_n181), .o1(new_n183));
  oaoi03aa1n06x5               g088(.a(\a[16] ), .b(\b[15] ), .c(new_n165), .o1(new_n184));
  nanb02aa1n02x5               g089(.a(new_n184), .b(new_n183), .out0(new_n185));
  aoi012aa1n12x5               g090(.a(new_n185), .b(new_n180), .c(new_n178), .o1(new_n186));
  oai012aa1d24x5               g091(.a(new_n186), .b(new_n175), .c(new_n179), .o1(new_n187));
  xorb03aa1n02x5               g092(.a(new_n187), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  nor002aa1d32x5               g093(.a(\b[16] ), .b(\a[17] ), .o1(new_n189));
  nand42aa1d28x5               g094(.a(\b[16] ), .b(\a[17] ), .o1(new_n190));
  tech160nm_fiaoi012aa1n05x5   g095(.a(new_n189), .b(new_n187), .c(new_n190), .o1(new_n191));
  xnrb03aa1n03x5               g096(.a(new_n191), .b(\b[17] ), .c(\a[18] ), .out0(\s[18] ));
  xnrc02aa1n12x5               g097(.a(\b[3] ), .b(\a[4] ), .out0(new_n193));
  aoi113aa1n03x7               g098(.a(new_n193), .b(new_n101), .c(new_n107), .d(new_n105), .e(new_n108), .o1(new_n194));
  norb02aa1n02x7               g099(.a(new_n116), .b(new_n121), .out0(new_n195));
  nand23aa1n03x5               g100(.a(new_n195), .b(new_n120), .c(new_n114), .o1(new_n196));
  oab012aa1n04x5               g101(.a(new_n119), .b(new_n111), .c(new_n112), .out0(new_n197));
  oab012aa1n09x5               g102(.a(new_n197), .b(new_n115), .c(new_n122), .out0(new_n198));
  oai012aa1n12x5               g103(.a(new_n198), .b(new_n196), .c(new_n194), .o1(new_n199));
  nano23aa1n03x7               g104(.a(new_n154), .b(new_n158), .c(new_n153), .d(new_n155), .out0(new_n200));
  nand23aa1n03x5               g105(.a(new_n200), .b(new_n167), .c(new_n182), .o1(new_n201));
  nano32aa1n09x5               g106(.a(new_n201), .b(new_n125), .c(new_n98), .d(new_n145), .out0(new_n202));
  aoi013aa1n03x5               g107(.a(new_n184), .b(new_n182), .c(new_n167), .d(new_n181), .o1(new_n203));
  oai012aa1n12x5               g108(.a(new_n203), .b(new_n147), .c(new_n201), .o1(new_n204));
  nor002aa1d32x5               g109(.a(\b[17] ), .b(\a[18] ), .o1(new_n205));
  nand42aa1d28x5               g110(.a(\b[17] ), .b(\a[18] ), .o1(new_n206));
  nano23aa1d15x5               g111(.a(new_n189), .b(new_n205), .c(new_n206), .d(new_n190), .out0(new_n207));
  aoai13aa1n06x5               g112(.a(new_n207), .b(new_n204), .c(new_n199), .d(new_n202), .o1(new_n208));
  inv000aa1d42x5               g113(.a(new_n189), .o1(new_n209));
  oaoi03aa1n12x5               g114(.a(\a[18] ), .b(\b[17] ), .c(new_n209), .o1(new_n210));
  inv000aa1d42x5               g115(.a(new_n210), .o1(new_n211));
  xorc02aa1n12x5               g116(.a(\a[19] ), .b(\b[18] ), .out0(new_n212));
  xnbna2aa1n03x5               g117(.a(new_n212), .b(new_n208), .c(new_n211), .out0(\s[19] ));
  xnrc02aa1n02x5               g118(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  orn002aa1n24x5               g119(.a(\a[19] ), .b(\b[18] ), .o(new_n215));
  aoai13aa1n06x5               g120(.a(new_n212), .b(new_n210), .c(new_n187), .d(new_n207), .o1(new_n216));
  nor022aa1n16x5               g121(.a(\b[19] ), .b(\a[20] ), .o1(new_n217));
  nand42aa1d28x5               g122(.a(\b[19] ), .b(\a[20] ), .o1(new_n218));
  nanb02aa1n02x5               g123(.a(new_n217), .b(new_n218), .out0(new_n219));
  tech160nm_fiaoi012aa1n05x5   g124(.a(new_n219), .b(new_n216), .c(new_n215), .o1(new_n220));
  nanp02aa1n02x5               g125(.a(\b[18] ), .b(\a[19] ), .o1(new_n221));
  nand42aa1n06x5               g126(.a(new_n215), .b(new_n221), .o1(new_n222));
  tech160nm_fiaoi012aa1n05x5   g127(.a(new_n222), .b(new_n208), .c(new_n211), .o1(new_n223));
  nano22aa1n03x7               g128(.a(new_n223), .b(new_n215), .c(new_n219), .out0(new_n224));
  norp02aa1n03x5               g129(.a(new_n220), .b(new_n224), .o1(\s[20] ));
  nano22aa1n09x5               g130(.a(new_n217), .b(new_n206), .c(new_n218), .out0(new_n226));
  nand23aa1n09x5               g131(.a(new_n207), .b(new_n212), .c(new_n226), .o1(new_n227));
  inv000aa1d42x5               g132(.a(new_n227), .o1(new_n228));
  aoai13aa1n06x5               g133(.a(new_n228), .b(new_n204), .c(new_n199), .d(new_n202), .o1(new_n229));
  norb03aa1n12x5               g134(.a(new_n206), .b(new_n189), .c(new_n205), .out0(new_n230));
  nanb03aa1n09x5               g135(.a(new_n217), .b(new_n218), .c(new_n206), .out0(new_n231));
  tech160nm_fioaoi03aa1n05x5   g136(.a(\a[20] ), .b(\b[19] ), .c(new_n215), .o1(new_n232));
  inv000aa1n04x5               g137(.a(new_n232), .o1(new_n233));
  oai013aa1d12x5               g138(.a(new_n233), .b(new_n230), .c(new_n231), .d(new_n222), .o1(new_n234));
  inv000aa1d42x5               g139(.a(new_n234), .o1(new_n235));
  nor002aa1d32x5               g140(.a(\b[20] ), .b(\a[21] ), .o1(new_n236));
  nand42aa1d28x5               g141(.a(\b[20] ), .b(\a[21] ), .o1(new_n237));
  norb02aa1n02x5               g142(.a(new_n237), .b(new_n236), .out0(new_n238));
  xnbna2aa1n03x5               g143(.a(new_n238), .b(new_n229), .c(new_n235), .out0(\s[21] ));
  inv000aa1d42x5               g144(.a(new_n236), .o1(new_n240));
  aoai13aa1n06x5               g145(.a(new_n238), .b(new_n234), .c(new_n187), .d(new_n228), .o1(new_n241));
  nor002aa1d32x5               g146(.a(\b[21] ), .b(\a[22] ), .o1(new_n242));
  nand42aa1d28x5               g147(.a(\b[21] ), .b(\a[22] ), .o1(new_n243));
  nanb02aa1n02x5               g148(.a(new_n242), .b(new_n243), .out0(new_n244));
  tech160nm_fiaoi012aa1n05x5   g149(.a(new_n244), .b(new_n241), .c(new_n240), .o1(new_n245));
  aobi12aa1n06x5               g150(.a(new_n238), .b(new_n229), .c(new_n235), .out0(new_n246));
  nano22aa1n03x7               g151(.a(new_n246), .b(new_n240), .c(new_n244), .out0(new_n247));
  norp02aa1n03x5               g152(.a(new_n245), .b(new_n247), .o1(\s[22] ));
  nona23aa1n02x4               g153(.a(new_n243), .b(new_n237), .c(new_n236), .d(new_n242), .out0(new_n249));
  nano32aa1n02x4               g154(.a(new_n249), .b(new_n207), .c(new_n212), .d(new_n226), .out0(new_n250));
  aoai13aa1n06x5               g155(.a(new_n250), .b(new_n204), .c(new_n199), .d(new_n202), .o1(new_n251));
  nano23aa1d15x5               g156(.a(new_n236), .b(new_n242), .c(new_n243), .d(new_n237), .out0(new_n252));
  inv040aa1n04x5               g157(.a(new_n242), .o1(new_n253));
  oai112aa1n06x5               g158(.a(new_n253), .b(new_n243), .c(\b[20] ), .d(\a[21] ), .o1(new_n254));
  aoi022aa1n12x5               g159(.a(new_n234), .b(new_n252), .c(new_n243), .d(new_n254), .o1(new_n255));
  xorc02aa1n12x5               g160(.a(\a[23] ), .b(\b[22] ), .out0(new_n256));
  xnbna2aa1n03x5               g161(.a(new_n256), .b(new_n251), .c(new_n255), .out0(\s[23] ));
  orn002aa1n03x5               g162(.a(\a[23] ), .b(\b[22] ), .o(new_n258));
  inv040aa1n02x5               g163(.a(new_n255), .o1(new_n259));
  aoai13aa1n06x5               g164(.a(new_n256), .b(new_n259), .c(new_n187), .d(new_n250), .o1(new_n260));
  nor042aa1n03x5               g165(.a(\b[23] ), .b(\a[24] ), .o1(new_n261));
  nand42aa1n20x5               g166(.a(\b[23] ), .b(\a[24] ), .o1(new_n262));
  nanb02aa1n02x5               g167(.a(new_n261), .b(new_n262), .out0(new_n263));
  aoi012aa1n06x5               g168(.a(new_n263), .b(new_n260), .c(new_n258), .o1(new_n264));
  xnrc02aa1n02x5               g169(.a(\b[22] ), .b(\a[23] ), .out0(new_n265));
  aoi012aa1n03x5               g170(.a(new_n265), .b(new_n251), .c(new_n255), .o1(new_n266));
  nano22aa1n03x5               g171(.a(new_n266), .b(new_n258), .c(new_n263), .out0(new_n267));
  norp02aa1n03x5               g172(.a(new_n264), .b(new_n267), .o1(\s[24] ));
  nano22aa1n03x7               g173(.a(new_n261), .b(new_n243), .c(new_n262), .out0(new_n269));
  nand23aa1n06x5               g174(.a(new_n252), .b(new_n256), .c(new_n269), .o1(new_n270));
  nor042aa1n02x5               g175(.a(new_n227), .b(new_n270), .o1(new_n271));
  aoai13aa1n06x5               g176(.a(new_n271), .b(new_n204), .c(new_n199), .d(new_n202), .o1(new_n272));
  nanb03aa1n03x5               g177(.a(new_n230), .b(new_n226), .c(new_n212), .out0(new_n273));
  oaoi03aa1n03x5               g178(.a(\a[24] ), .b(\b[23] ), .c(new_n258), .o1(new_n274));
  aoi013aa1n06x4               g179(.a(new_n274), .b(new_n254), .c(new_n269), .d(new_n256), .o1(new_n275));
  aoai13aa1n06x5               g180(.a(new_n275), .b(new_n270), .c(new_n273), .d(new_n233), .o1(new_n276));
  inv000aa1n02x5               g181(.a(new_n276), .o1(new_n277));
  xnrc02aa1n12x5               g182(.a(\b[24] ), .b(\a[25] ), .out0(new_n278));
  inv000aa1d42x5               g183(.a(new_n278), .o1(new_n279));
  xnbna2aa1n03x5               g184(.a(new_n279), .b(new_n272), .c(new_n277), .out0(\s[25] ));
  nor042aa1n03x5               g185(.a(\b[24] ), .b(\a[25] ), .o1(new_n281));
  inv000aa1d42x5               g186(.a(new_n281), .o1(new_n282));
  aoai13aa1n06x5               g187(.a(new_n279), .b(new_n276), .c(new_n187), .d(new_n271), .o1(new_n283));
  tech160nm_fixnrc02aa1n04x5   g188(.a(\b[25] ), .b(\a[26] ), .out0(new_n284));
  tech160nm_fiaoi012aa1n05x5   g189(.a(new_n284), .b(new_n283), .c(new_n282), .o1(new_n285));
  tech160nm_fiaoi012aa1n05x5   g190(.a(new_n278), .b(new_n272), .c(new_n277), .o1(new_n286));
  nano22aa1n03x7               g191(.a(new_n286), .b(new_n282), .c(new_n284), .out0(new_n287));
  nor002aa1n02x5               g192(.a(new_n285), .b(new_n287), .o1(\s[26] ));
  nor042aa1n06x5               g193(.a(new_n284), .b(new_n278), .o1(new_n289));
  inv000aa1n04x5               g194(.a(new_n289), .o1(new_n290));
  nor043aa1d12x5               g195(.a(new_n227), .b(new_n270), .c(new_n290), .o1(new_n291));
  aoai13aa1n12x5               g196(.a(new_n291), .b(new_n204), .c(new_n199), .d(new_n202), .o1(new_n292));
  oao003aa1n12x5               g197(.a(\a[26] ), .b(\b[25] ), .c(new_n282), .carry(new_n293));
  inv000aa1d42x5               g198(.a(new_n293), .o1(new_n294));
  aoi012aa1n06x5               g199(.a(new_n294), .b(new_n276), .c(new_n289), .o1(new_n295));
  xorc02aa1n12x5               g200(.a(\a[27] ), .b(\b[26] ), .out0(new_n296));
  xnbna2aa1n03x5               g201(.a(new_n296), .b(new_n292), .c(new_n295), .out0(\s[27] ));
  nor042aa1n03x5               g202(.a(\b[26] ), .b(\a[27] ), .o1(new_n298));
  inv040aa1n03x5               g203(.a(new_n298), .o1(new_n299));
  norp03aa1n06x5               g204(.a(new_n230), .b(new_n231), .c(new_n222), .o1(new_n300));
  norb03aa1n03x5               g205(.a(new_n269), .b(new_n249), .c(new_n265), .out0(new_n301));
  oai012aa1n02x7               g206(.a(new_n301), .b(new_n300), .c(new_n232), .o1(new_n302));
  aoai13aa1n06x5               g207(.a(new_n293), .b(new_n290), .c(new_n302), .d(new_n275), .o1(new_n303));
  aoai13aa1n03x5               g208(.a(new_n296), .b(new_n303), .c(new_n187), .d(new_n291), .o1(new_n304));
  xnrc02aa1n02x5               g209(.a(\b[27] ), .b(\a[28] ), .out0(new_n305));
  tech160nm_fiaoi012aa1n02p5x5 g210(.a(new_n305), .b(new_n304), .c(new_n299), .o1(new_n306));
  aobi12aa1n02x7               g211(.a(new_n296), .b(new_n292), .c(new_n295), .out0(new_n307));
  nano22aa1n03x5               g212(.a(new_n307), .b(new_n299), .c(new_n305), .out0(new_n308));
  norp02aa1n03x5               g213(.a(new_n306), .b(new_n308), .o1(\s[28] ));
  norb02aa1n02x5               g214(.a(new_n296), .b(new_n305), .out0(new_n310));
  aoai13aa1n03x5               g215(.a(new_n310), .b(new_n303), .c(new_n187), .d(new_n291), .o1(new_n311));
  oao003aa1n03x5               g216(.a(\a[28] ), .b(\b[27] ), .c(new_n299), .carry(new_n312));
  xnrc02aa1n02x5               g217(.a(\b[28] ), .b(\a[29] ), .out0(new_n313));
  tech160nm_fiaoi012aa1n02p5x5 g218(.a(new_n313), .b(new_n311), .c(new_n312), .o1(new_n314));
  aobi12aa1n02x7               g219(.a(new_n310), .b(new_n292), .c(new_n295), .out0(new_n315));
  nano22aa1n03x5               g220(.a(new_n315), .b(new_n312), .c(new_n313), .out0(new_n316));
  norp02aa1n03x5               g221(.a(new_n314), .b(new_n316), .o1(\s[29] ));
  and002aa1n02x5               g222(.a(\b[0] ), .b(\a[1] ), .o(new_n318));
  xnrb03aa1n02x5               g223(.a(new_n318), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g224(.a(new_n296), .b(new_n313), .c(new_n305), .out0(new_n320));
  aoai13aa1n03x5               g225(.a(new_n320), .b(new_n303), .c(new_n187), .d(new_n291), .o1(new_n321));
  oao003aa1n02x5               g226(.a(\a[29] ), .b(\b[28] ), .c(new_n312), .carry(new_n322));
  xnrc02aa1n02x5               g227(.a(\b[29] ), .b(\a[30] ), .out0(new_n323));
  tech160nm_fiaoi012aa1n02p5x5 g228(.a(new_n323), .b(new_n321), .c(new_n322), .o1(new_n324));
  aobi12aa1n02x7               g229(.a(new_n320), .b(new_n292), .c(new_n295), .out0(new_n325));
  nano22aa1n03x5               g230(.a(new_n325), .b(new_n322), .c(new_n323), .out0(new_n326));
  norp02aa1n03x5               g231(.a(new_n324), .b(new_n326), .o1(\s[30] ));
  xnrc02aa1n02x5               g232(.a(\b[30] ), .b(\a[31] ), .out0(new_n328));
  nona32aa1n02x4               g233(.a(new_n296), .b(new_n323), .c(new_n313), .d(new_n305), .out0(new_n329));
  inv000aa1n02x5               g234(.a(new_n329), .o1(new_n330));
  aoai13aa1n03x5               g235(.a(new_n330), .b(new_n303), .c(new_n187), .d(new_n291), .o1(new_n331));
  oao003aa1n02x5               g236(.a(\a[30] ), .b(\b[29] ), .c(new_n322), .carry(new_n332));
  aoi012aa1n02x7               g237(.a(new_n328), .b(new_n331), .c(new_n332), .o1(new_n333));
  tech160nm_fiaoi012aa1n03p5x5 g238(.a(new_n329), .b(new_n292), .c(new_n295), .o1(new_n334));
  nano22aa1n03x7               g239(.a(new_n334), .b(new_n328), .c(new_n332), .out0(new_n335));
  norp02aa1n03x5               g240(.a(new_n333), .b(new_n335), .o1(\s[31] ));
  xnbna2aa1n03x5               g241(.a(new_n106), .b(new_n107), .c(new_n108), .out0(\s[3] ));
  aoi013aa1n02x4               g242(.a(new_n101), .b(new_n107), .c(new_n108), .d(new_n105), .o1(new_n338));
  oaib12aa1n02x5               g243(.a(new_n110), .b(new_n338), .c(new_n193), .out0(\s[4] ));
  inv000aa1d42x5               g244(.a(new_n102), .o1(new_n340));
  xnrc02aa1n02x5               g245(.a(\b[4] ), .b(\a[5] ), .out0(new_n341));
  xnbna2aa1n03x5               g246(.a(new_n341), .b(new_n110), .c(new_n340), .out0(\s[5] ));
  norp02aa1n02x5               g247(.a(\b[4] ), .b(\a[5] ), .o1(new_n343));
  inv000aa1d42x5               g248(.a(new_n343), .o1(new_n344));
  nona22aa1n03x5               g249(.a(new_n110), .b(new_n341), .c(new_n102), .out0(new_n345));
  xorc02aa1n02x5               g250(.a(\a[6] ), .b(\b[5] ), .out0(new_n346));
  nanp02aa1n03x5               g251(.a(new_n345), .b(new_n122), .o1(new_n347));
  aoai13aa1n02x5               g252(.a(new_n347), .b(new_n346), .c(new_n344), .d(new_n345), .o1(\s[6] ));
  aob012aa1n03x5               g253(.a(new_n347), .b(\b[5] ), .c(\a[6] ), .out0(new_n349));
  xnrb03aa1n02x5               g254(.a(new_n349), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  oaoi03aa1n03x5               g255(.a(\a[7] ), .b(\b[6] ), .c(new_n349), .o1(new_n351));
  xorb03aa1n02x5               g256(.a(new_n351), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xnrc02aa1n02x5               g257(.a(new_n175), .b(new_n125), .out0(\s[9] ));
endmodule


