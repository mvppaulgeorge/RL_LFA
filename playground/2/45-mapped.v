// Benchmark "adder" written by ABC on Wed Jul 17 13:21:34 2024

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
    new_n132, new_n133, new_n134, new_n135, new_n136, new_n137, new_n138,
    new_n139, new_n141, new_n142, new_n143, new_n144, new_n146, new_n147,
    new_n148, new_n149, new_n150, new_n151, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n160, new_n161, new_n163,
    new_n164, new_n165, new_n167, new_n168, new_n169, new_n170, new_n171,
    new_n172, new_n173, new_n175, new_n176, new_n177, new_n178, new_n179,
    new_n180, new_n181, new_n182, new_n183, new_n184, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n193, new_n195,
    new_n196, new_n197, new_n198, new_n199, new_n201, new_n202, new_n203,
    new_n204, new_n205, new_n206, new_n207, new_n208, new_n211, new_n212,
    new_n213, new_n214, new_n215, new_n216, new_n218, new_n219, new_n220,
    new_n221, new_n222, new_n223, new_n224, new_n226, new_n227, new_n228,
    new_n229, new_n230, new_n232, new_n233, new_n234, new_n235, new_n236,
    new_n237, new_n238, new_n239, new_n241, new_n242, new_n243, new_n244,
    new_n245, new_n247, new_n248, new_n249, new_n250, new_n251, new_n252,
    new_n253, new_n254, new_n255, new_n256, new_n257, new_n259, new_n260,
    new_n261, new_n262, new_n263, new_n264, new_n266, new_n267, new_n268,
    new_n269, new_n270, new_n271, new_n273, new_n274, new_n275, new_n276,
    new_n277, new_n278, new_n279, new_n280, new_n281, new_n282, new_n283,
    new_n285, new_n286, new_n287, new_n288, new_n289, new_n290, new_n291,
    new_n292, new_n295, new_n296, new_n297, new_n298, new_n299, new_n300,
    new_n301, new_n302, new_n303, new_n305, new_n306, new_n307, new_n308,
    new_n309, new_n310, new_n313, new_n315, new_n317, new_n318, new_n321;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  xnrc02aa1n12x5               g001(.a(\b[9] ), .b(\a[10] ), .out0(new_n97));
  nor042aa1d18x5               g002(.a(\b[8] ), .b(\a[9] ), .o1(new_n98));
  nanp02aa1n04x5               g003(.a(\b[8] ), .b(\a[9] ), .o1(new_n99));
  nanp02aa1n02x5               g004(.a(\b[3] ), .b(\a[4] ), .o1(new_n100));
  nand22aa1n03x5               g005(.a(\b[0] ), .b(\a[1] ), .o1(new_n101));
  nanp02aa1n04x5               g006(.a(\b[1] ), .b(\a[2] ), .o1(new_n102));
  nor042aa1n02x5               g007(.a(\b[1] ), .b(\a[2] ), .o1(new_n103));
  nona22aa1n06x5               g008(.a(new_n102), .b(new_n103), .c(new_n101), .out0(new_n104));
  nor002aa1n12x5               g009(.a(\b[2] ), .b(\a[3] ), .o1(new_n105));
  nand02aa1d12x5               g010(.a(\b[2] ), .b(\a[3] ), .o1(new_n106));
  nano22aa1n03x7               g011(.a(new_n105), .b(new_n102), .c(new_n106), .out0(new_n107));
  oab012aa1n12x5               g012(.a(new_n105), .b(\a[4] ), .c(\b[3] ), .out0(new_n108));
  inv000aa1d42x5               g013(.a(new_n108), .o1(new_n109));
  aoai13aa1n12x5               g014(.a(new_n100), .b(new_n109), .c(new_n107), .d(new_n104), .o1(new_n110));
  nor002aa1n04x5               g015(.a(\b[7] ), .b(\a[8] ), .o1(new_n111));
  nand42aa1n10x5               g016(.a(\b[7] ), .b(\a[8] ), .o1(new_n112));
  nor042aa1d18x5               g017(.a(\b[6] ), .b(\a[7] ), .o1(new_n113));
  tech160nm_finand02aa1n03p5x5 g018(.a(\b[6] ), .b(\a[7] ), .o1(new_n114));
  nano23aa1n03x7               g019(.a(new_n111), .b(new_n113), .c(new_n114), .d(new_n112), .out0(new_n115));
  nor002aa1n02x5               g020(.a(\b[5] ), .b(\a[6] ), .o1(new_n116));
  nand42aa1n20x5               g021(.a(\b[5] ), .b(\a[6] ), .o1(new_n117));
  norb02aa1n03x4               g022(.a(new_n117), .b(new_n116), .out0(new_n118));
  xnrc02aa1n12x5               g023(.a(\b[4] ), .b(\a[5] ), .out0(new_n119));
  inv000aa1d42x5               g024(.a(new_n119), .o1(new_n120));
  nanp03aa1n02x5               g025(.a(new_n120), .b(new_n115), .c(new_n118), .o1(new_n121));
  orn002aa1n03x5               g026(.a(\a[6] ), .b(\b[5] ), .o(new_n122));
  oai112aa1n03x5               g027(.a(new_n122), .b(new_n117), .c(\b[4] ), .d(\a[5] ), .o1(new_n123));
  inv000aa1n06x5               g028(.a(new_n113), .o1(new_n124));
  oaoi03aa1n12x5               g029(.a(\a[8] ), .b(\b[7] ), .c(new_n124), .o1(new_n125));
  aoi013aa1n02x4               g030(.a(new_n125), .b(new_n115), .c(new_n117), .d(new_n123), .o1(new_n126));
  oai012aa1n04x7               g031(.a(new_n126), .b(new_n110), .c(new_n121), .o1(new_n127));
  aoai13aa1n02x5               g032(.a(new_n97), .b(new_n98), .c(new_n127), .d(new_n99), .o1(new_n128));
  norb02aa1n12x5               g033(.a(new_n106), .b(new_n105), .out0(new_n129));
  oai112aa1n04x5               g034(.a(new_n129), .b(new_n102), .c(new_n103), .d(new_n101), .o1(new_n130));
  aoi022aa1n09x5               g035(.a(new_n130), .b(new_n108), .c(\b[3] ), .d(\a[4] ), .o1(new_n131));
  norb02aa1n03x4               g036(.a(new_n112), .b(new_n111), .out0(new_n132));
  nanb02aa1n02x5               g037(.a(new_n113), .b(new_n114), .out0(new_n133));
  nano23aa1n06x5               g038(.a(new_n119), .b(new_n133), .c(new_n132), .d(new_n118), .out0(new_n134));
  nanp03aa1n03x5               g039(.a(new_n115), .b(new_n117), .c(new_n123), .o1(new_n135));
  inv000aa1d42x5               g040(.a(new_n125), .o1(new_n136));
  nand22aa1n03x5               g041(.a(new_n135), .b(new_n136), .o1(new_n137));
  aoai13aa1n02x5               g042(.a(new_n99), .b(new_n137), .c(new_n131), .d(new_n134), .o1(new_n138));
  nona22aa1n02x5               g043(.a(new_n138), .b(new_n98), .c(new_n97), .out0(new_n139));
  nanp02aa1n02x5               g044(.a(new_n128), .b(new_n139), .o1(\s[10] ));
  nanp02aa1n02x5               g045(.a(\b[9] ), .b(\a[10] ), .o1(new_n141));
  nand42aa1d28x5               g046(.a(\b[10] ), .b(\a[11] ), .o1(new_n142));
  nor042aa1n06x5               g047(.a(\b[10] ), .b(\a[11] ), .o1(new_n143));
  norb02aa1n02x5               g048(.a(new_n142), .b(new_n143), .out0(new_n144));
  xobna2aa1n03x5               g049(.a(new_n144), .b(new_n139), .c(new_n141), .out0(\s[11] ));
  aoi013aa1n02x4               g050(.a(new_n143), .b(new_n139), .c(new_n141), .d(new_n144), .o1(new_n146));
  nor042aa1n04x5               g051(.a(\b[11] ), .b(\a[12] ), .o1(new_n147));
  nand42aa1d28x5               g052(.a(\b[11] ), .b(\a[12] ), .o1(new_n148));
  norb02aa1n02x5               g053(.a(new_n148), .b(new_n147), .out0(new_n149));
  nona22aa1n09x5               g054(.a(new_n148), .b(new_n147), .c(new_n143), .out0(new_n150));
  aoi013aa1n02x4               g055(.a(new_n150), .b(new_n139), .c(new_n144), .d(new_n141), .o1(new_n151));
  oabi12aa1n02x5               g056(.a(new_n151), .b(new_n146), .c(new_n149), .out0(\s[12] ));
  nano23aa1d15x5               g057(.a(new_n147), .b(new_n143), .c(new_n148), .d(new_n142), .out0(new_n153));
  nanb02aa1n06x5               g058(.a(new_n98), .b(new_n99), .out0(new_n154));
  nona22aa1d30x5               g059(.a(new_n153), .b(new_n97), .c(new_n154), .out0(new_n155));
  inv000aa1d42x5               g060(.a(new_n155), .o1(new_n156));
  aoai13aa1n06x5               g061(.a(new_n156), .b(new_n137), .c(new_n131), .d(new_n134), .o1(new_n157));
  inv000aa1n02x5               g062(.a(new_n98), .o1(new_n158));
  tech160nm_fioaoi03aa1n02p5x5 g063(.a(\a[10] ), .b(\b[9] ), .c(new_n158), .o1(new_n159));
  aoi022aa1n09x5               g064(.a(new_n153), .b(new_n159), .c(new_n150), .d(new_n148), .o1(new_n160));
  xnrc02aa1n12x5               g065(.a(\b[12] ), .b(\a[13] ), .out0(new_n161));
  xobna2aa1n03x5               g066(.a(new_n161), .b(new_n157), .c(new_n160), .out0(\s[13] ));
  orn002aa1n02x5               g067(.a(\a[13] ), .b(\b[12] ), .o(new_n163));
  tech160nm_fiao0012aa1n02p5x5 g068(.a(new_n161), .b(new_n157), .c(new_n160), .o(new_n164));
  xnrc02aa1n12x5               g069(.a(\b[13] ), .b(\a[14] ), .out0(new_n165));
  xobna2aa1n03x5               g070(.a(new_n165), .b(new_n164), .c(new_n163), .out0(\s[14] ));
  nor042aa1n09x5               g071(.a(new_n165), .b(new_n161), .o1(new_n167));
  inv000aa1d42x5               g072(.a(new_n167), .o1(new_n168));
  norp02aa1n03x5               g073(.a(\b[12] ), .b(\a[13] ), .o1(new_n169));
  norp02aa1n02x5               g074(.a(\b[13] ), .b(\a[14] ), .o1(new_n170));
  nanp02aa1n02x5               g075(.a(\b[13] ), .b(\a[14] ), .o1(new_n171));
  tech160nm_fioai012aa1n04x5   g076(.a(new_n171), .b(new_n170), .c(new_n169), .o1(new_n172));
  aoai13aa1n06x5               g077(.a(new_n172), .b(new_n168), .c(new_n157), .d(new_n160), .o1(new_n173));
  xorb03aa1n02x5               g078(.a(new_n173), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  xnrc02aa1n12x5               g079(.a(\b[14] ), .b(\a[15] ), .out0(new_n175));
  inv000aa1d42x5               g080(.a(new_n175), .o1(new_n176));
  norp02aa1n02x5               g081(.a(\b[14] ), .b(\a[15] ), .o1(new_n177));
  inv040aa1d32x5               g082(.a(\a[16] ), .o1(new_n178));
  inv030aa1d32x5               g083(.a(\b[15] ), .o1(new_n179));
  nand42aa1n02x5               g084(.a(new_n179), .b(new_n178), .o1(new_n180));
  nanp02aa1n02x5               g085(.a(\b[15] ), .b(\a[16] ), .o1(new_n181));
  nand02aa1n03x5               g086(.a(new_n180), .b(new_n181), .o1(new_n182));
  aoai13aa1n02x5               g087(.a(new_n182), .b(new_n177), .c(new_n173), .d(new_n176), .o1(new_n183));
  oai112aa1n02x5               g088(.a(new_n180), .b(new_n181), .c(\b[14] ), .d(\a[15] ), .o1(new_n184));
  aoai13aa1n02x5               g089(.a(new_n183), .b(new_n184), .c(new_n176), .d(new_n173), .o1(\s[16] ));
  nor042aa1n06x5               g090(.a(new_n175), .b(new_n182), .o1(new_n186));
  nano22aa1d15x5               g091(.a(new_n155), .b(new_n167), .c(new_n186), .out0(new_n187));
  aoai13aa1n12x5               g092(.a(new_n187), .b(new_n137), .c(new_n131), .d(new_n134), .o1(new_n188));
  nanp02aa1n04x5               g093(.a(new_n167), .b(new_n186), .o1(new_n189));
  oaoi03aa1n02x5               g094(.a(new_n178), .b(new_n179), .c(new_n177), .o1(new_n190));
  oai013aa1n06x5               g095(.a(new_n190), .b(new_n175), .c(new_n172), .d(new_n182), .o1(new_n191));
  oab012aa1d15x5               g096(.a(new_n191), .b(new_n160), .c(new_n189), .out0(new_n192));
  xorc02aa1n02x5               g097(.a(\a[17] ), .b(\b[16] ), .out0(new_n193));
  xnbna2aa1n03x5               g098(.a(new_n193), .b(new_n188), .c(new_n192), .out0(\s[17] ));
  inv000aa1d42x5               g099(.a(\a[17] ), .o1(new_n195));
  nanb02aa1n02x5               g100(.a(\b[16] ), .b(new_n195), .out0(new_n196));
  oabi12aa1n06x5               g101(.a(new_n191), .b(new_n160), .c(new_n189), .out0(new_n197));
  aoai13aa1n02x5               g102(.a(new_n193), .b(new_n197), .c(new_n127), .d(new_n187), .o1(new_n198));
  xnrc02aa1n02x5               g103(.a(\b[17] ), .b(\a[18] ), .out0(new_n199));
  xobna2aa1n03x5               g104(.a(new_n199), .b(new_n198), .c(new_n196), .out0(\s[18] ));
  inv000aa1d42x5               g105(.a(\a[18] ), .o1(new_n201));
  xroi22aa1d04x5               g106(.a(new_n195), .b(\b[16] ), .c(new_n201), .d(\b[17] ), .out0(new_n202));
  aoai13aa1n04x5               g107(.a(new_n202), .b(new_n197), .c(new_n127), .d(new_n187), .o1(new_n203));
  oaih22aa1n06x5               g108(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n204));
  oaib12aa1n18x5               g109(.a(new_n204), .b(new_n201), .c(\b[17] ), .out0(new_n205));
  nor022aa1n16x5               g110(.a(\b[18] ), .b(\a[19] ), .o1(new_n206));
  nand02aa1d06x5               g111(.a(\b[18] ), .b(\a[19] ), .o1(new_n207));
  norb02aa1n02x5               g112(.a(new_n207), .b(new_n206), .out0(new_n208));
  xnbna2aa1n03x5               g113(.a(new_n208), .b(new_n203), .c(new_n205), .out0(\s[19] ));
  xnrc02aa1n02x5               g114(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  aobi12aa1n06x5               g115(.a(new_n208), .b(new_n203), .c(new_n205), .out0(new_n211));
  nor002aa1n12x5               g116(.a(\b[19] ), .b(\a[20] ), .o1(new_n212));
  nand22aa1n09x5               g117(.a(\b[19] ), .b(\a[20] ), .o1(new_n213));
  nanb02aa1n02x5               g118(.a(new_n212), .b(new_n213), .out0(new_n214));
  oai012aa1n03x5               g119(.a(new_n214), .b(new_n211), .c(new_n206), .o1(new_n215));
  norb03aa1n02x5               g120(.a(new_n213), .b(new_n206), .c(new_n212), .out0(new_n216));
  oaib12aa1n03x5               g121(.a(new_n215), .b(new_n211), .c(new_n216), .out0(\s[20] ));
  nano23aa1n06x5               g122(.a(new_n206), .b(new_n212), .c(new_n213), .d(new_n207), .out0(new_n218));
  nanb03aa1n09x5               g123(.a(new_n199), .b(new_n218), .c(new_n193), .out0(new_n219));
  nona23aa1d18x5               g124(.a(new_n213), .b(new_n207), .c(new_n206), .d(new_n212), .out0(new_n220));
  oai012aa1n12x5               g125(.a(new_n213), .b(new_n212), .c(new_n206), .o1(new_n221));
  oai012aa1d24x5               g126(.a(new_n221), .b(new_n220), .c(new_n205), .o1(new_n222));
  inv000aa1d42x5               g127(.a(new_n222), .o1(new_n223));
  aoai13aa1n06x5               g128(.a(new_n223), .b(new_n219), .c(new_n188), .d(new_n192), .o1(new_n224));
  xorb03aa1n02x5               g129(.a(new_n224), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  xorc02aa1n02x5               g130(.a(\a[21] ), .b(\b[20] ), .out0(new_n226));
  norp02aa1n02x5               g131(.a(\b[20] ), .b(\a[21] ), .o1(new_n227));
  xnrc02aa1n02x5               g132(.a(\b[21] ), .b(\a[22] ), .out0(new_n228));
  aoai13aa1n03x5               g133(.a(new_n228), .b(new_n227), .c(new_n224), .d(new_n226), .o1(new_n229));
  oabi12aa1n02x5               g134(.a(new_n228), .b(\a[21] ), .c(\b[20] ), .out0(new_n230));
  aoai13aa1n03x5               g135(.a(new_n229), .b(new_n230), .c(new_n226), .d(new_n224), .o1(\s[22] ));
  inv000aa1d42x5               g136(.a(\a[21] ), .o1(new_n232));
  inv040aa1d32x5               g137(.a(\a[22] ), .o1(new_n233));
  xroi22aa1d06x4               g138(.a(new_n232), .b(\b[20] ), .c(new_n233), .d(\b[21] ), .out0(new_n234));
  nand23aa1n03x5               g139(.a(new_n234), .b(new_n202), .c(new_n218), .o1(new_n235));
  inv000aa1d42x5               g140(.a(\b[21] ), .o1(new_n236));
  oao003aa1n06x5               g141(.a(new_n233), .b(new_n236), .c(new_n227), .carry(new_n237));
  aoi012aa1n02x5               g142(.a(new_n237), .b(new_n222), .c(new_n234), .o1(new_n238));
  aoai13aa1n06x5               g143(.a(new_n238), .b(new_n235), .c(new_n188), .d(new_n192), .o1(new_n239));
  xorb03aa1n02x5               g144(.a(new_n239), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  tech160nm_fixorc02aa1n05x5   g145(.a(\a[23] ), .b(\b[22] ), .out0(new_n241));
  norp02aa1n02x5               g146(.a(\b[22] ), .b(\a[23] ), .o1(new_n242));
  xnrc02aa1n12x5               g147(.a(\b[23] ), .b(\a[24] ), .out0(new_n243));
  aoai13aa1n03x5               g148(.a(new_n243), .b(new_n242), .c(new_n239), .d(new_n241), .o1(new_n244));
  oabi12aa1n02x5               g149(.a(new_n243), .b(\a[23] ), .c(\b[22] ), .out0(new_n245));
  aoai13aa1n03x5               g150(.a(new_n244), .b(new_n245), .c(new_n241), .d(new_n239), .o1(\s[24] ));
  norb02aa1n06x5               g151(.a(new_n241), .b(new_n243), .out0(new_n247));
  nanb03aa1n02x5               g152(.a(new_n219), .b(new_n247), .c(new_n234), .out0(new_n248));
  oaoi03aa1n02x5               g153(.a(\a[18] ), .b(\b[17] ), .c(new_n196), .o1(new_n249));
  inv040aa1n02x5               g154(.a(new_n221), .o1(new_n250));
  aoai13aa1n03x5               g155(.a(new_n234), .b(new_n250), .c(new_n218), .d(new_n249), .o1(new_n251));
  inv000aa1n02x5               g156(.a(new_n237), .o1(new_n252));
  inv000aa1d42x5               g157(.a(new_n247), .o1(new_n253));
  aob012aa1n02x5               g158(.a(new_n245), .b(\b[23] ), .c(\a[24] ), .out0(new_n254));
  aoai13aa1n02x5               g159(.a(new_n254), .b(new_n253), .c(new_n251), .d(new_n252), .o1(new_n255));
  inv040aa1n03x5               g160(.a(new_n255), .o1(new_n256));
  aoai13aa1n04x5               g161(.a(new_n256), .b(new_n248), .c(new_n188), .d(new_n192), .o1(new_n257));
  xorb03aa1n02x5               g162(.a(new_n257), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g163(.a(\b[24] ), .b(\a[25] ), .o1(new_n259));
  tech160nm_fixorc02aa1n05x5   g164(.a(\a[25] ), .b(\b[24] ), .out0(new_n260));
  xnrc02aa1n12x5               g165(.a(\b[25] ), .b(\a[26] ), .out0(new_n261));
  aoai13aa1n03x5               g166(.a(new_n261), .b(new_n259), .c(new_n257), .d(new_n260), .o1(new_n262));
  norp02aa1n02x5               g167(.a(new_n261), .b(new_n259), .o1(new_n263));
  aob012aa1n03x5               g168(.a(new_n263), .b(new_n257), .c(new_n260), .out0(new_n264));
  nanp02aa1n03x5               g169(.a(new_n262), .b(new_n264), .o1(\s[26] ));
  norb02aa1n09x5               g170(.a(new_n260), .b(new_n261), .out0(new_n266));
  nano22aa1n03x5               g171(.a(new_n235), .b(new_n247), .c(new_n266), .out0(new_n267));
  aoai13aa1n06x5               g172(.a(new_n267), .b(new_n197), .c(new_n127), .d(new_n187), .o1(new_n268));
  nanp02aa1n03x5               g173(.a(new_n255), .b(new_n266), .o1(new_n269));
  tech160nm_fiao0012aa1n02p5x5 g174(.a(new_n263), .b(\a[26] ), .c(\b[25] ), .o(new_n270));
  nand23aa1n03x5               g175(.a(new_n268), .b(new_n269), .c(new_n270), .o1(new_n271));
  xorb03aa1n02x5               g176(.a(new_n271), .b(\b[26] ), .c(\a[27] ), .out0(\s[27] ));
  inv000aa1n02x5               g177(.a(new_n267), .o1(new_n273));
  aoi012aa1n12x5               g178(.a(new_n273), .b(new_n188), .c(new_n192), .o1(new_n274));
  aoai13aa1n02x7               g179(.a(new_n247), .b(new_n237), .c(new_n222), .d(new_n234), .o1(new_n275));
  inv000aa1d42x5               g180(.a(new_n266), .o1(new_n276));
  aoai13aa1n06x5               g181(.a(new_n270), .b(new_n276), .c(new_n275), .d(new_n254), .o1(new_n277));
  norp02aa1n02x5               g182(.a(\b[26] ), .b(\a[27] ), .o1(new_n278));
  xorc02aa1n02x5               g183(.a(\a[27] ), .b(\b[26] ), .out0(new_n279));
  oaoi13aa1n03x5               g184(.a(new_n278), .b(new_n279), .c(new_n277), .d(new_n274), .o1(new_n280));
  xorc02aa1n02x5               g185(.a(\a[28] ), .b(\b[27] ), .out0(new_n281));
  oaih12aa1n02x5               g186(.a(new_n279), .b(new_n277), .c(new_n274), .o1(new_n282));
  oai112aa1n02x7               g187(.a(new_n282), .b(new_n281), .c(\b[26] ), .d(\a[27] ), .o1(new_n283));
  oaih12aa1n02x5               g188(.a(new_n283), .b(new_n280), .c(new_n281), .o1(\s[28] ));
  xorc02aa1n12x5               g189(.a(\a[29] ), .b(\b[28] ), .out0(new_n285));
  inv000aa1d42x5               g190(.a(new_n285), .o1(new_n286));
  and002aa1n02x5               g191(.a(new_n281), .b(new_n279), .o(new_n287));
  orn002aa1n02x5               g192(.a(\a[27] ), .b(\b[26] ), .o(new_n288));
  oaoi03aa1n02x5               g193(.a(\a[28] ), .b(\b[27] ), .c(new_n288), .o1(new_n289));
  aoai13aa1n02x5               g194(.a(new_n286), .b(new_n289), .c(new_n271), .d(new_n287), .o1(new_n290));
  oaih12aa1n02x5               g195(.a(new_n287), .b(new_n277), .c(new_n274), .o1(new_n291));
  nona22aa1n02x5               g196(.a(new_n291), .b(new_n289), .c(new_n286), .out0(new_n292));
  nanp02aa1n03x5               g197(.a(new_n290), .b(new_n292), .o1(\s[29] ));
  xorb03aa1n02x5               g198(.a(new_n101), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n02x4               g199(.a(new_n286), .b(new_n279), .c(new_n281), .out0(new_n295));
  inv000aa1d42x5               g200(.a(\b[28] ), .o1(new_n296));
  oaib12aa1n02x5               g201(.a(new_n289), .b(new_n296), .c(\a[29] ), .out0(new_n297));
  oaib12aa1n02x5               g202(.a(new_n297), .b(\a[29] ), .c(new_n296), .out0(new_n298));
  oaoi13aa1n03x5               g203(.a(new_n298), .b(new_n295), .c(new_n277), .d(new_n274), .o1(new_n299));
  xorc02aa1n02x5               g204(.a(\a[30] ), .b(\b[29] ), .out0(new_n300));
  oaih12aa1n02x5               g205(.a(new_n295), .b(new_n277), .c(new_n274), .o1(new_n301));
  norb02aa1n02x5               g206(.a(new_n300), .b(new_n298), .out0(new_n302));
  tech160nm_finand02aa1n03p5x5 g207(.a(new_n301), .b(new_n302), .o1(new_n303));
  oaih12aa1n02x5               g208(.a(new_n303), .b(new_n299), .c(new_n300), .o1(\s[30] ));
  nano32aa1n02x4               g209(.a(new_n286), .b(new_n300), .c(new_n279), .d(new_n281), .out0(new_n305));
  aoi012aa1n02x5               g210(.a(new_n302), .b(\a[30] ), .c(\b[29] ), .o1(new_n306));
  xnrc02aa1n02x5               g211(.a(\b[30] ), .b(\a[31] ), .out0(new_n307));
  aoai13aa1n02x5               g212(.a(new_n307), .b(new_n306), .c(new_n271), .d(new_n305), .o1(new_n308));
  oaih12aa1n02x5               g213(.a(new_n305), .b(new_n277), .c(new_n274), .o1(new_n309));
  nona22aa1n02x5               g214(.a(new_n309), .b(new_n306), .c(new_n307), .out0(new_n310));
  nanp02aa1n03x5               g215(.a(new_n308), .b(new_n310), .o1(\s[31] ));
  xobna2aa1n03x5               g216(.a(new_n129), .b(new_n104), .c(new_n102), .out0(\s[3] ));
  aoi012aa1n02x5               g217(.a(new_n105), .b(new_n107), .c(new_n104), .o1(new_n313));
  xnrb03aa1n02x5               g218(.a(new_n313), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  inv000aa1d42x5               g219(.a(\a[5] ), .o1(new_n315));
  xorb03aa1n02x5               g220(.a(new_n110), .b(\b[4] ), .c(new_n315), .out0(\s[5] ));
  oaoi03aa1n02x5               g221(.a(\a[5] ), .b(\b[4] ), .c(new_n110), .o1(new_n317));
  oabi12aa1n02x5               g222(.a(new_n123), .b(new_n110), .c(new_n119), .out0(new_n318));
  oaib12aa1n02x5               g223(.a(new_n318), .b(new_n118), .c(new_n317), .out0(\s[6] ));
  xnbna2aa1n03x5               g224(.a(new_n133), .b(new_n318), .c(new_n117), .out0(\s[7] ));
  nanb03aa1n02x5               g225(.a(new_n133), .b(new_n318), .c(new_n117), .out0(new_n321));
  xnbna2aa1n03x5               g226(.a(new_n132), .b(new_n321), .c(new_n124), .out0(\s[8] ));
  xorb03aa1n02x5               g227(.a(new_n127), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


