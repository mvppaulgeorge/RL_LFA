// Benchmark "adder" written by ABC on Wed Jul 17 13:03:05 2024

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
    new_n132, new_n133, new_n134, new_n136, new_n137, new_n138, new_n139,
    new_n141, new_n142, new_n143, new_n144, new_n145, new_n146, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n157, new_n158, new_n159, new_n160, new_n161, new_n162, new_n163,
    new_n165, new_n166, new_n167, new_n169, new_n170, new_n171, new_n172,
    new_n173, new_n174, new_n175, new_n176, new_n177, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n189, new_n190, new_n191, new_n192, new_n194, new_n195, new_n196,
    new_n197, new_n198, new_n199, new_n202, new_n203, new_n204, new_n205,
    new_n206, new_n207, new_n208, new_n209, new_n210, new_n212, new_n213,
    new_n214, new_n215, new_n216, new_n217, new_n218, new_n220, new_n221,
    new_n222, new_n223, new_n224, new_n225, new_n226, new_n228, new_n229,
    new_n230, new_n231, new_n232, new_n233, new_n234, new_n235, new_n236,
    new_n237, new_n239, new_n240, new_n241, new_n242, new_n243, new_n245,
    new_n246, new_n247, new_n248, new_n249, new_n250, new_n251, new_n253,
    new_n254, new_n255, new_n256, new_n257, new_n258, new_n260, new_n261,
    new_n262, new_n263, new_n264, new_n265, new_n266, new_n268, new_n269,
    new_n270, new_n271, new_n272, new_n273, new_n274, new_n275, new_n276,
    new_n277, new_n278, new_n279, new_n280, new_n282, new_n283, new_n284,
    new_n285, new_n286, new_n287, new_n288, new_n291, new_n292, new_n293,
    new_n294, new_n295, new_n296, new_n297, new_n298, new_n299, new_n300,
    new_n302, new_n303, new_n304, new_n305, new_n306, new_n307, new_n309,
    new_n311, new_n312, new_n315, new_n317, new_n318, new_n319, new_n320,
    new_n321, new_n323;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  xnrc02aa1n12x5               g001(.a(\b[9] ), .b(\a[10] ), .out0(new_n97));
  nor042aa1n03x5               g002(.a(\b[8] ), .b(\a[9] ), .o1(new_n98));
  nanp02aa1n02x5               g003(.a(\b[8] ), .b(\a[9] ), .o1(new_n99));
  nor042aa1n04x5               g004(.a(\b[3] ), .b(\a[4] ), .o1(new_n100));
  tech160nm_finand02aa1n05x5   g005(.a(\b[3] ), .b(\a[4] ), .o1(new_n101));
  norb02aa1n02x5               g006(.a(new_n101), .b(new_n100), .out0(new_n102));
  nand42aa1n06x5               g007(.a(\b[1] ), .b(\a[2] ), .o1(new_n103));
  nor042aa1n06x5               g008(.a(\b[2] ), .b(\a[3] ), .o1(new_n104));
  tech160nm_finand02aa1n03p5x5 g009(.a(\b[2] ), .b(\a[3] ), .o1(new_n105));
  nano22aa1n02x4               g010(.a(new_n104), .b(new_n103), .c(new_n105), .out0(new_n106));
  nand22aa1n02x5               g011(.a(\b[0] ), .b(\a[1] ), .o1(new_n107));
  nor002aa1n02x5               g012(.a(\b[1] ), .b(\a[2] ), .o1(new_n108));
  nona22aa1n02x4               g013(.a(new_n103), .b(new_n108), .c(new_n107), .out0(new_n109));
  nanp03aa1n02x5               g014(.a(new_n106), .b(new_n109), .c(new_n102), .o1(new_n110));
  oai012aa1n12x5               g015(.a(new_n101), .b(new_n104), .c(new_n100), .o1(new_n111));
  nor002aa1n03x5               g016(.a(\b[7] ), .b(\a[8] ), .o1(new_n112));
  nand22aa1n03x5               g017(.a(\b[7] ), .b(\a[8] ), .o1(new_n113));
  nor002aa1n20x5               g018(.a(\b[6] ), .b(\a[7] ), .o1(new_n114));
  nand42aa1n03x5               g019(.a(\b[6] ), .b(\a[7] ), .o1(new_n115));
  nano23aa1n03x5               g020(.a(new_n112), .b(new_n114), .c(new_n115), .d(new_n113), .out0(new_n116));
  tech160nm_fixorc02aa1n02p5x5 g021(.a(\a[6] ), .b(\b[5] ), .out0(new_n117));
  tech160nm_fixorc02aa1n04x5   g022(.a(\a[5] ), .b(\b[4] ), .out0(new_n118));
  nanp03aa1n02x5               g023(.a(new_n116), .b(new_n117), .c(new_n118), .o1(new_n119));
  orn002aa1n03x5               g024(.a(\a[5] ), .b(\b[4] ), .o(new_n120));
  oaoi03aa1n02x5               g025(.a(\a[6] ), .b(\b[5] ), .c(new_n120), .o1(new_n121));
  oaih12aa1n02x5               g026(.a(new_n113), .b(new_n114), .c(new_n112), .o1(new_n122));
  aobi12aa1n02x5               g027(.a(new_n122), .b(new_n116), .c(new_n121), .out0(new_n123));
  aoai13aa1n06x5               g028(.a(new_n123), .b(new_n119), .c(new_n110), .d(new_n111), .o1(new_n124));
  aoai13aa1n02x5               g029(.a(new_n97), .b(new_n98), .c(new_n124), .d(new_n99), .o1(new_n125));
  nanb02aa1n06x5               g030(.a(new_n100), .b(new_n101), .out0(new_n126));
  nanb03aa1n09x5               g031(.a(new_n104), .b(new_n105), .c(new_n103), .out0(new_n127));
  norb03aa1n03x5               g032(.a(new_n103), .b(new_n108), .c(new_n107), .out0(new_n128));
  oai013aa1d12x5               g033(.a(new_n111), .b(new_n128), .c(new_n127), .d(new_n126), .o1(new_n129));
  nona23aa1n03x5               g034(.a(new_n115), .b(new_n113), .c(new_n112), .d(new_n114), .out0(new_n130));
  nano22aa1n03x7               g035(.a(new_n130), .b(new_n117), .c(new_n118), .out0(new_n131));
  oaib12aa1n09x5               g036(.a(new_n122), .b(new_n130), .c(new_n121), .out0(new_n132));
  aoai13aa1n02x5               g037(.a(new_n99), .b(new_n132), .c(new_n129), .d(new_n131), .o1(new_n133));
  nona22aa1n02x5               g038(.a(new_n133), .b(new_n98), .c(new_n97), .out0(new_n134));
  nanp02aa1n02x5               g039(.a(new_n125), .b(new_n134), .o1(\s[10] ));
  nanp02aa1n02x5               g040(.a(\b[9] ), .b(\a[10] ), .o1(new_n136));
  tech160nm_finand02aa1n03p5x5 g041(.a(\b[10] ), .b(\a[11] ), .o1(new_n137));
  nor002aa1n03x5               g042(.a(\b[10] ), .b(\a[11] ), .o1(new_n138));
  norb02aa1n03x5               g043(.a(new_n137), .b(new_n138), .out0(new_n139));
  xobna2aa1n03x5               g044(.a(new_n139), .b(new_n134), .c(new_n136), .out0(\s[11] ));
  aoi013aa1n02x4               g045(.a(new_n138), .b(new_n134), .c(new_n136), .d(new_n139), .o1(new_n141));
  nor042aa1n02x5               g046(.a(\b[11] ), .b(\a[12] ), .o1(new_n142));
  nand42aa1n04x5               g047(.a(\b[11] ), .b(\a[12] ), .o1(new_n143));
  norb02aa1n02x5               g048(.a(new_n143), .b(new_n142), .out0(new_n144));
  nona22aa1n03x5               g049(.a(new_n143), .b(new_n142), .c(new_n138), .out0(new_n145));
  aoi013aa1n02x4               g050(.a(new_n145), .b(new_n134), .c(new_n139), .d(new_n136), .o1(new_n146));
  oabi12aa1n02x7               g051(.a(new_n146), .b(new_n141), .c(new_n144), .out0(\s[12] ));
  nanb02aa1n03x5               g052(.a(new_n98), .b(new_n99), .out0(new_n148));
  nano23aa1n02x4               g053(.a(new_n148), .b(new_n97), .c(new_n139), .d(new_n144), .out0(new_n149));
  aoai13aa1n06x5               g054(.a(new_n149), .b(new_n132), .c(new_n129), .d(new_n131), .o1(new_n150));
  nano23aa1n03x7               g055(.a(new_n142), .b(new_n138), .c(new_n143), .d(new_n137), .out0(new_n151));
  inv000aa1n02x5               g056(.a(new_n98), .o1(new_n152));
  oaoi03aa1n02x5               g057(.a(\a[10] ), .b(\b[9] ), .c(new_n152), .o1(new_n153));
  aoi022aa1n09x5               g058(.a(new_n151), .b(new_n153), .c(new_n145), .d(new_n143), .o1(new_n154));
  xorc02aa1n12x5               g059(.a(\a[13] ), .b(\b[12] ), .out0(new_n155));
  xnbna2aa1n03x5               g060(.a(new_n155), .b(new_n150), .c(new_n154), .out0(\s[13] ));
  nor042aa1n09x5               g061(.a(\b[12] ), .b(\a[13] ), .o1(new_n157));
  inv000aa1d42x5               g062(.a(new_n157), .o1(new_n158));
  nanp02aa1n03x5               g063(.a(new_n150), .b(new_n154), .o1(new_n159));
  nanp02aa1n02x5               g064(.a(new_n159), .b(new_n155), .o1(new_n160));
  nor042aa1n02x5               g065(.a(\b[13] ), .b(\a[14] ), .o1(new_n161));
  nand42aa1n03x5               g066(.a(\b[13] ), .b(\a[14] ), .o1(new_n162));
  norb02aa1n06x5               g067(.a(new_n162), .b(new_n161), .out0(new_n163));
  xnbna2aa1n03x5               g068(.a(new_n163), .b(new_n160), .c(new_n158), .out0(\s[14] ));
  nanp02aa1n02x5               g069(.a(new_n155), .b(new_n163), .o1(new_n165));
  tech160nm_fioai012aa1n03p5x5 g070(.a(new_n162), .b(new_n161), .c(new_n157), .o1(new_n166));
  aoai13aa1n06x5               g071(.a(new_n166), .b(new_n165), .c(new_n150), .d(new_n154), .o1(new_n167));
  xorb03aa1n02x5               g072(.a(new_n167), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  nor042aa1n06x5               g073(.a(\b[14] ), .b(\a[15] ), .o1(new_n169));
  nanp02aa1n04x5               g074(.a(\b[14] ), .b(\a[15] ), .o1(new_n170));
  nanb02aa1n02x5               g075(.a(new_n169), .b(new_n170), .out0(new_n171));
  norb02aa1n02x5               g076(.a(new_n167), .b(new_n171), .out0(new_n172));
  nor042aa1n04x5               g077(.a(\b[15] ), .b(\a[16] ), .o1(new_n173));
  nanp02aa1n04x5               g078(.a(\b[15] ), .b(\a[16] ), .o1(new_n174));
  nanb02aa1n03x5               g079(.a(new_n173), .b(new_n174), .out0(new_n175));
  aoai13aa1n02x5               g080(.a(new_n175), .b(new_n169), .c(new_n167), .d(new_n170), .o1(new_n176));
  nona22aa1n02x4               g081(.a(new_n174), .b(new_n173), .c(new_n169), .out0(new_n177));
  oai012aa1n02x5               g082(.a(new_n176), .b(new_n172), .c(new_n177), .o1(\s[16] ));
  norp02aa1n02x5               g083(.a(new_n97), .b(new_n148), .o1(new_n179));
  nano23aa1n06x5               g084(.a(new_n169), .b(new_n173), .c(new_n174), .d(new_n170), .out0(new_n180));
  nand03aa1n04x5               g085(.a(new_n180), .b(new_n155), .c(new_n163), .o1(new_n181));
  nano22aa1n03x7               g086(.a(new_n181), .b(new_n179), .c(new_n151), .out0(new_n182));
  aoai13aa1n12x5               g087(.a(new_n182), .b(new_n132), .c(new_n129), .d(new_n131), .o1(new_n183));
  oai012aa1n02x5               g088(.a(new_n174), .b(new_n173), .c(new_n169), .o1(new_n184));
  oai013aa1n06x5               g089(.a(new_n184), .b(new_n166), .c(new_n171), .d(new_n175), .o1(new_n185));
  oab012aa1d15x5               g090(.a(new_n185), .b(new_n154), .c(new_n181), .out0(new_n186));
  nanp02aa1n06x5               g091(.a(new_n183), .b(new_n186), .o1(new_n187));
  xorb03aa1n02x5               g092(.a(new_n187), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv040aa1d32x5               g093(.a(\a[18] ), .o1(new_n189));
  inv040aa1d30x5               g094(.a(\a[17] ), .o1(new_n190));
  inv000aa1d42x5               g095(.a(\b[16] ), .o1(new_n191));
  oaoi03aa1n03x5               g096(.a(new_n190), .b(new_n191), .c(new_n187), .o1(new_n192));
  xorb03aa1n02x5               g097(.a(new_n192), .b(\b[17] ), .c(new_n189), .out0(\s[18] ));
  xroi22aa1d06x4               g098(.a(new_n190), .b(\b[16] ), .c(new_n189), .d(\b[17] ), .out0(new_n194));
  inv000aa1d42x5               g099(.a(new_n194), .o1(new_n195));
  nor002aa1n02x5               g100(.a(\b[17] ), .b(\a[18] ), .o1(new_n196));
  aoi112aa1n09x5               g101(.a(\b[16] ), .b(\a[17] ), .c(\a[18] ), .d(\b[17] ), .o1(new_n197));
  nor002aa1n03x5               g102(.a(new_n197), .b(new_n196), .o1(new_n198));
  aoai13aa1n04x5               g103(.a(new_n198), .b(new_n195), .c(new_n183), .d(new_n186), .o1(new_n199));
  xorb03aa1n02x5               g104(.a(new_n199), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g105(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor002aa1d32x5               g106(.a(\b[18] ), .b(\a[19] ), .o1(new_n202));
  nand02aa1n20x5               g107(.a(\b[18] ), .b(\a[19] ), .o1(new_n203));
  aoi012aa1n02x5               g108(.a(new_n202), .b(new_n199), .c(new_n203), .o1(new_n204));
  nor002aa1d32x5               g109(.a(\b[19] ), .b(\a[20] ), .o1(new_n205));
  nand42aa1d28x5               g110(.a(\b[19] ), .b(\a[20] ), .o1(new_n206));
  norb02aa1n03x4               g111(.a(new_n206), .b(new_n205), .out0(new_n207));
  norb02aa1n02x7               g112(.a(new_n203), .b(new_n202), .out0(new_n208));
  nona22aa1n02x4               g113(.a(new_n206), .b(new_n205), .c(new_n202), .out0(new_n209));
  tech160nm_fiao0012aa1n03p5x5 g114(.a(new_n209), .b(new_n199), .c(new_n208), .o(new_n210));
  oaih12aa1n02x5               g115(.a(new_n210), .b(new_n204), .c(new_n207), .o1(\s[20] ));
  nona23aa1d24x5               g116(.a(new_n206), .b(new_n203), .c(new_n202), .d(new_n205), .out0(new_n212));
  inv000aa1d42x5               g117(.a(new_n212), .o1(new_n213));
  nanp02aa1n02x5               g118(.a(new_n194), .b(new_n213), .o1(new_n214));
  oaih12aa1n06x5               g119(.a(new_n206), .b(new_n205), .c(new_n202), .o1(new_n215));
  oai012aa1n12x5               g120(.a(new_n215), .b(new_n212), .c(new_n198), .o1(new_n216));
  inv000aa1d42x5               g121(.a(new_n216), .o1(new_n217));
  aoai13aa1n04x5               g122(.a(new_n217), .b(new_n214), .c(new_n183), .d(new_n186), .o1(new_n218));
  xorb03aa1n02x5               g123(.a(new_n218), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor042aa1n06x5               g124(.a(\b[20] ), .b(\a[21] ), .o1(new_n220));
  xnrc02aa1n12x5               g125(.a(\b[20] ), .b(\a[21] ), .out0(new_n221));
  inv000aa1d42x5               g126(.a(new_n221), .o1(new_n222));
  aoi012aa1n02x5               g127(.a(new_n220), .b(new_n218), .c(new_n222), .o1(new_n223));
  xorc02aa1n12x5               g128(.a(\a[22] ), .b(\b[21] ), .out0(new_n224));
  norb02aa1n02x5               g129(.a(new_n224), .b(new_n220), .out0(new_n225));
  aob012aa1n03x5               g130(.a(new_n225), .b(new_n218), .c(new_n222), .out0(new_n226));
  oaih12aa1n02x5               g131(.a(new_n226), .b(new_n223), .c(new_n224), .o1(\s[22] ));
  nona23aa1n08x5               g132(.a(new_n194), .b(new_n224), .c(new_n221), .d(new_n212), .out0(new_n228));
  oai112aa1n03x5               g133(.a(new_n208), .b(new_n207), .c(new_n197), .d(new_n196), .o1(new_n229));
  nanb02aa1n06x5               g134(.a(new_n221), .b(new_n224), .out0(new_n230));
  inv000aa1d42x5               g135(.a(\a[22] ), .o1(new_n231));
  inv040aa1d32x5               g136(.a(\b[21] ), .o1(new_n232));
  oao003aa1n03x5               g137(.a(new_n231), .b(new_n232), .c(new_n220), .carry(new_n233));
  inv020aa1n02x5               g138(.a(new_n233), .o1(new_n234));
  aoai13aa1n12x5               g139(.a(new_n234), .b(new_n230), .c(new_n229), .d(new_n215), .o1(new_n235));
  inv000aa1d42x5               g140(.a(new_n235), .o1(new_n236));
  aoai13aa1n04x5               g141(.a(new_n236), .b(new_n228), .c(new_n183), .d(new_n186), .o1(new_n237));
  xorb03aa1n02x5               g142(.a(new_n237), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  xorc02aa1n12x5               g143(.a(\a[23] ), .b(\b[22] ), .out0(new_n239));
  norp02aa1n02x5               g144(.a(\b[22] ), .b(\a[23] ), .o1(new_n240));
  xnrc02aa1n02x5               g145(.a(\b[23] ), .b(\a[24] ), .out0(new_n241));
  aoai13aa1n03x5               g146(.a(new_n241), .b(new_n240), .c(new_n237), .d(new_n239), .o1(new_n242));
  oabi12aa1n02x5               g147(.a(new_n241), .b(\a[23] ), .c(\b[22] ), .out0(new_n243));
  aoai13aa1n03x5               g148(.a(new_n242), .b(new_n243), .c(new_n239), .d(new_n237), .o1(\s[24] ));
  norb02aa1n02x5               g149(.a(new_n239), .b(new_n241), .out0(new_n245));
  nona23aa1n02x4               g150(.a(new_n194), .b(new_n245), .c(new_n230), .d(new_n212), .out0(new_n246));
  inv000aa1d42x5               g151(.a(\a[24] ), .o1(new_n247));
  inv000aa1d42x5               g152(.a(\b[23] ), .o1(new_n248));
  oao003aa1n02x5               g153(.a(new_n247), .b(new_n248), .c(new_n240), .carry(new_n249));
  aoi012aa1n06x5               g154(.a(new_n249), .b(new_n235), .c(new_n245), .o1(new_n250));
  aoai13aa1n06x5               g155(.a(new_n250), .b(new_n246), .c(new_n183), .d(new_n186), .o1(new_n251));
  xorb03aa1n02x5               g156(.a(new_n251), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  xnrc02aa1n12x5               g157(.a(\b[24] ), .b(\a[25] ), .out0(new_n253));
  inv000aa1d42x5               g158(.a(new_n253), .o1(new_n254));
  norp02aa1n02x5               g159(.a(\b[24] ), .b(\a[25] ), .o1(new_n255));
  xnrc02aa1n06x5               g160(.a(\b[25] ), .b(\a[26] ), .out0(new_n256));
  aoai13aa1n03x5               g161(.a(new_n256), .b(new_n255), .c(new_n251), .d(new_n254), .o1(new_n257));
  oabi12aa1n02x5               g162(.a(new_n256), .b(\a[25] ), .c(\b[24] ), .out0(new_n258));
  aoai13aa1n03x5               g163(.a(new_n257), .b(new_n258), .c(new_n254), .d(new_n251), .o1(\s[26] ));
  oabi12aa1n03x5               g164(.a(new_n185), .b(new_n154), .c(new_n181), .out0(new_n260));
  nor002aa1n04x5               g165(.a(new_n256), .b(new_n253), .o1(new_n261));
  nano22aa1n03x7               g166(.a(new_n228), .b(new_n245), .c(new_n261), .out0(new_n262));
  aoai13aa1n06x5               g167(.a(new_n262), .b(new_n260), .c(new_n124), .d(new_n182), .o1(new_n263));
  aoai13aa1n04x5               g168(.a(new_n261), .b(new_n249), .c(new_n235), .d(new_n245), .o1(new_n264));
  aob012aa1n02x5               g169(.a(new_n258), .b(\b[25] ), .c(\a[26] ), .out0(new_n265));
  nanp03aa1n06x5               g170(.a(new_n263), .b(new_n264), .c(new_n265), .o1(new_n266));
  xorb03aa1n02x5               g171(.a(new_n266), .b(\b[26] ), .c(\a[27] ), .out0(\s[27] ));
  nor042aa1n02x5               g172(.a(\b[26] ), .b(\a[27] ), .o1(new_n268));
  xorc02aa1n02x5               g173(.a(\a[27] ), .b(\b[26] ), .out0(new_n269));
  xnrc02aa1n12x5               g174(.a(\b[27] ), .b(\a[28] ), .out0(new_n270));
  aoai13aa1n03x5               g175(.a(new_n270), .b(new_n268), .c(new_n266), .d(new_n269), .o1(new_n271));
  aobi12aa1n06x5               g176(.a(new_n262), .b(new_n183), .c(new_n186), .out0(new_n272));
  norb02aa1n02x5               g177(.a(new_n224), .b(new_n221), .out0(new_n273));
  aoai13aa1n03x5               g178(.a(new_n245), .b(new_n233), .c(new_n216), .d(new_n273), .o1(new_n274));
  inv000aa1d42x5               g179(.a(new_n249), .o1(new_n275));
  inv000aa1d42x5               g180(.a(new_n261), .o1(new_n276));
  aoai13aa1n06x5               g181(.a(new_n265), .b(new_n276), .c(new_n274), .d(new_n275), .o1(new_n277));
  oaih12aa1n02x5               g182(.a(new_n269), .b(new_n277), .c(new_n272), .o1(new_n278));
  nor042aa1n04x5               g183(.a(new_n270), .b(new_n268), .o1(new_n279));
  tech160nm_finand02aa1n03p5x5 g184(.a(new_n278), .b(new_n279), .o1(new_n280));
  nanp02aa1n03x5               g185(.a(new_n271), .b(new_n280), .o1(\s[28] ));
  tech160nm_fixorc02aa1n03p5x5 g186(.a(\a[29] ), .b(\b[28] ), .out0(new_n282));
  inv000aa1d42x5               g187(.a(new_n282), .o1(new_n283));
  norb02aa1n02x5               g188(.a(new_n269), .b(new_n270), .out0(new_n284));
  aoi012aa1n06x5               g189(.a(new_n279), .b(\a[28] ), .c(\b[27] ), .o1(new_n285));
  aoai13aa1n03x5               g190(.a(new_n283), .b(new_n285), .c(new_n266), .d(new_n284), .o1(new_n286));
  oaih12aa1n02x5               g191(.a(new_n284), .b(new_n277), .c(new_n272), .o1(new_n287));
  nona22aa1n02x5               g192(.a(new_n287), .b(new_n285), .c(new_n283), .out0(new_n288));
  nanp02aa1n03x5               g193(.a(new_n286), .b(new_n288), .o1(\s[29] ));
  xorb03aa1n02x5               g194(.a(new_n107), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n02x4               g195(.a(new_n270), .b(new_n269), .c(new_n282), .out0(new_n291));
  inv000aa1d42x5               g196(.a(\a[29] ), .o1(new_n292));
  inv000aa1d42x5               g197(.a(\b[28] ), .o1(new_n293));
  tech160nm_fioaoi03aa1n03p5x5 g198(.a(new_n292), .b(new_n293), .c(new_n285), .o1(new_n294));
  inv000aa1d42x5               g199(.a(new_n294), .o1(new_n295));
  tech160nm_fixorc02aa1n03p5x5 g200(.a(\a[30] ), .b(\b[29] ), .out0(new_n296));
  inv000aa1d42x5               g201(.a(new_n296), .o1(new_n297));
  aoai13aa1n03x5               g202(.a(new_n297), .b(new_n295), .c(new_n266), .d(new_n291), .o1(new_n298));
  oaih12aa1n02x5               g203(.a(new_n291), .b(new_n277), .c(new_n272), .o1(new_n299));
  nona22aa1n02x5               g204(.a(new_n299), .b(new_n295), .c(new_n297), .out0(new_n300));
  nanp02aa1n03x5               g205(.a(new_n298), .b(new_n300), .o1(\s[30] ));
  xnrc02aa1n02x5               g206(.a(\b[30] ), .b(\a[31] ), .out0(new_n302));
  nano23aa1n02x4               g207(.a(new_n297), .b(new_n270), .c(new_n269), .d(new_n282), .out0(new_n303));
  oaoi03aa1n02x5               g208(.a(\a[30] ), .b(\b[29] ), .c(new_n294), .o1(new_n304));
  aoai13aa1n03x5               g209(.a(new_n302), .b(new_n304), .c(new_n266), .d(new_n303), .o1(new_n305));
  oaih12aa1n02x5               g210(.a(new_n303), .b(new_n277), .c(new_n272), .o1(new_n306));
  nona22aa1n02x5               g211(.a(new_n306), .b(new_n304), .c(new_n302), .out0(new_n307));
  nanp02aa1n03x5               g212(.a(new_n305), .b(new_n307), .o1(\s[31] ));
  norb02aa1n02x5               g213(.a(new_n105), .b(new_n104), .out0(new_n309));
  xobna2aa1n03x5               g214(.a(new_n309), .b(new_n109), .c(new_n103), .out0(\s[3] ));
  nona22aa1n02x4               g215(.a(new_n101), .b(new_n104), .c(new_n100), .out0(new_n311));
  aoai13aa1n02x5               g216(.a(new_n126), .b(new_n104), .c(new_n106), .d(new_n109), .o1(new_n312));
  aoai13aa1n02x5               g217(.a(new_n312), .b(new_n311), .c(new_n106), .d(new_n109), .o1(\s[4] ));
  xnbna2aa1n03x5               g218(.a(new_n118), .b(new_n110), .c(new_n111), .out0(\s[5] ));
  nanp02aa1n02x5               g219(.a(new_n129), .b(new_n118), .o1(new_n315));
  xnbna2aa1n03x5               g220(.a(new_n117), .b(new_n315), .c(new_n120), .out0(\s[6] ));
  inv000aa1d42x5               g221(.a(new_n114), .o1(new_n317));
  and002aa1n02x5               g222(.a(\b[5] ), .b(\a[6] ), .o(new_n318));
  nanp03aa1n02x5               g223(.a(new_n315), .b(new_n117), .c(new_n120), .o1(new_n319));
  aboi22aa1n03x5               g224(.a(new_n318), .b(new_n319), .c(new_n317), .d(new_n115), .out0(new_n320));
  nona23aa1n02x4               g225(.a(new_n319), .b(new_n115), .c(new_n114), .d(new_n318), .out0(new_n321));
  norb02aa1n02x5               g226(.a(new_n321), .b(new_n320), .out0(\s[7] ));
  nanb02aa1n02x5               g227(.a(new_n112), .b(new_n113), .out0(new_n323));
  xobna2aa1n03x5               g228(.a(new_n323), .b(new_n321), .c(new_n317), .out0(\s[8] ));
  xorb03aa1n02x5               g229(.a(new_n124), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


