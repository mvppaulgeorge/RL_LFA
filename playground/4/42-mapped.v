// Benchmark "adder" written by ABC on Wed Jul 17 14:21:25 2024

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
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n139, new_n140,
    new_n141, new_n142, new_n143, new_n144, new_n145, new_n146, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n158, new_n159, new_n160, new_n161, new_n163, new_n164,
    new_n165, new_n166, new_n167, new_n168, new_n169, new_n170, new_n171,
    new_n173, new_n174, new_n175, new_n176, new_n177, new_n178, new_n179,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n193, new_n194, new_n195,
    new_n197, new_n198, new_n199, new_n200, new_n201, new_n202, new_n203,
    new_n204, new_n205, new_n206, new_n207, new_n210, new_n211, new_n212,
    new_n213, new_n215, new_n216, new_n217, new_n218, new_n219, new_n220,
    new_n221, new_n222, new_n223, new_n225, new_n226, new_n227, new_n228,
    new_n229, new_n231, new_n232, new_n233, new_n234, new_n235, new_n236,
    new_n237, new_n238, new_n239, new_n240, new_n241, new_n242, new_n243,
    new_n245, new_n246, new_n247, new_n248, new_n249, new_n250, new_n252,
    new_n253, new_n254, new_n255, new_n256, new_n257, new_n258, new_n259,
    new_n260, new_n262, new_n263, new_n264, new_n265, new_n266, new_n267,
    new_n268, new_n269, new_n271, new_n272, new_n273, new_n274, new_n275,
    new_n276, new_n277, new_n278, new_n280, new_n281, new_n282, new_n283,
    new_n284, new_n285, new_n286, new_n287, new_n288, new_n289, new_n291,
    new_n292, new_n293, new_n294, new_n295, new_n296, new_n297, new_n300,
    new_n301, new_n302, new_n303, new_n304, new_n305, new_n306, new_n308,
    new_n309, new_n310, new_n311, new_n312, new_n313, new_n314, new_n317,
    new_n318, new_n319, new_n320, new_n323, new_n325, new_n326, new_n327,
    new_n329;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor042aa1n06x5               g001(.a(\b[9] ), .b(\a[10] ), .o1(new_n97));
  inv000aa1d42x5               g002(.a(new_n97), .o1(new_n98));
  nand42aa1n20x5               g003(.a(\b[9] ), .b(\a[10] ), .o1(new_n99));
  inv000aa1d42x5               g004(.a(\a[9] ), .o1(new_n100));
  inv000aa1d42x5               g005(.a(\b[8] ), .o1(new_n101));
  inv040aa1d32x5               g006(.a(\a[3] ), .o1(new_n102));
  inv000aa1d42x5               g007(.a(\b[2] ), .o1(new_n103));
  nand22aa1n04x5               g008(.a(new_n103), .b(new_n102), .o1(new_n104));
  nanp02aa1n02x5               g009(.a(\b[2] ), .b(\a[3] ), .o1(new_n105));
  nand02aa1n02x5               g010(.a(new_n104), .b(new_n105), .o1(new_n106));
  orn002aa1n02x7               g011(.a(\a[2] ), .b(\b[1] ), .o(new_n107));
  tech160nm_finand02aa1n05x5   g012(.a(\b[0] ), .b(\a[1] ), .o1(new_n108));
  aob012aa1n03x5               g013(.a(new_n108), .b(\b[1] ), .c(\a[2] ), .out0(new_n109));
  inv000aa1d42x5               g014(.a(\a[4] ), .o1(new_n110));
  aboi22aa1n03x5               g015(.a(\b[3] ), .b(new_n110), .c(new_n102), .d(new_n103), .out0(new_n111));
  aoai13aa1n06x5               g016(.a(new_n111), .b(new_n106), .c(new_n109), .d(new_n107), .o1(new_n112));
  xnrc02aa1n02x5               g017(.a(\b[4] ), .b(\a[5] ), .out0(new_n113));
  nor022aa1n06x5               g018(.a(\b[6] ), .b(\a[7] ), .o1(new_n114));
  nand42aa1n03x5               g019(.a(\b[6] ), .b(\a[7] ), .o1(new_n115));
  tech160nm_finand02aa1n03p5x5 g020(.a(\b[5] ), .b(\a[6] ), .o1(new_n116));
  nanb03aa1n06x5               g021(.a(new_n114), .b(new_n116), .c(new_n115), .out0(new_n117));
  nor042aa1n02x5               g022(.a(new_n117), .b(new_n113), .o1(new_n118));
  nand42aa1n03x5               g023(.a(\b[3] ), .b(\a[4] ), .o1(new_n119));
  nor002aa1n03x5               g024(.a(\b[7] ), .b(\a[8] ), .o1(new_n120));
  nanp02aa1n04x5               g025(.a(\b[7] ), .b(\a[8] ), .o1(new_n121));
  nor042aa1n02x5               g026(.a(\b[5] ), .b(\a[6] ), .o1(new_n122));
  nano23aa1n06x5               g027(.a(new_n122), .b(new_n120), .c(new_n119), .d(new_n121), .out0(new_n123));
  nand23aa1n03x5               g028(.a(new_n112), .b(new_n118), .c(new_n123), .o1(new_n124));
  nanb02aa1n12x5               g029(.a(new_n120), .b(new_n121), .out0(new_n125));
  oab012aa1n02x4               g030(.a(new_n122), .b(\a[5] ), .c(\b[4] ), .out0(new_n126));
  norp03aa1n02x5               g031(.a(new_n117), .b(new_n125), .c(new_n126), .o1(new_n127));
  aoi012aa1n02x5               g032(.a(new_n120), .b(new_n114), .c(new_n121), .o1(new_n128));
  norb02aa1n02x5               g033(.a(new_n128), .b(new_n127), .out0(new_n129));
  nanp02aa1n02x5               g034(.a(new_n124), .b(new_n129), .o1(new_n130));
  tech160nm_fioaoi03aa1n03p5x5 g035(.a(new_n100), .b(new_n101), .c(new_n130), .o1(new_n131));
  xnbna2aa1n03x5               g036(.a(new_n131), .b(new_n98), .c(new_n99), .out0(\s[10] ));
  inv000aa1d42x5               g037(.a(new_n99), .o1(new_n133));
  nona22aa1n02x4               g038(.a(new_n131), .b(new_n133), .c(new_n97), .out0(new_n134));
  nand42aa1n08x5               g039(.a(\b[10] ), .b(\a[11] ), .o1(new_n135));
  nor002aa1d32x5               g040(.a(\b[10] ), .b(\a[11] ), .o1(new_n136));
  norb02aa1n02x5               g041(.a(new_n135), .b(new_n136), .out0(new_n137));
  xobna2aa1n03x5               g042(.a(new_n137), .b(new_n134), .c(new_n99), .out0(\s[11] ));
  nanb03aa1d24x5               g043(.a(new_n136), .b(new_n99), .c(new_n135), .out0(new_n139));
  inv000aa1d42x5               g044(.a(new_n139), .o1(new_n140));
  nor042aa1n04x5               g045(.a(\b[11] ), .b(\a[12] ), .o1(new_n141));
  nand02aa1n06x5               g046(.a(\b[11] ), .b(\a[12] ), .o1(new_n142));
  nanb02aa1d24x5               g047(.a(new_n141), .b(new_n142), .out0(new_n143));
  inv000aa1d42x5               g048(.a(new_n143), .o1(new_n144));
  aoai13aa1n02x5               g049(.a(new_n144), .b(new_n136), .c(new_n134), .d(new_n140), .o1(new_n145));
  aoi112aa1n02x5               g050(.a(new_n136), .b(new_n144), .c(new_n134), .d(new_n140), .o1(new_n146));
  norb02aa1n02x5               g051(.a(new_n145), .b(new_n146), .out0(\s[12] ));
  oai013aa1n03x5               g052(.a(new_n128), .b(new_n117), .c(new_n125), .d(new_n126), .o1(new_n148));
  aoi013aa1n09x5               g053(.a(new_n148), .b(new_n112), .c(new_n118), .d(new_n123), .o1(new_n149));
  xorc02aa1n02x5               g054(.a(\a[9] ), .b(\b[8] ), .out0(new_n150));
  nano23aa1n02x4               g055(.a(new_n143), .b(new_n139), .c(new_n150), .d(new_n98), .out0(new_n151));
  aoi112aa1n09x5               g056(.a(new_n133), .b(new_n97), .c(new_n100), .d(new_n101), .o1(new_n152));
  aoi012aa1n02x5               g057(.a(new_n141), .b(new_n136), .c(new_n142), .o1(new_n153));
  oai013aa1d12x5               g058(.a(new_n153), .b(new_n152), .c(new_n139), .d(new_n143), .o1(new_n154));
  inv000aa1d42x5               g059(.a(new_n154), .o1(new_n155));
  oaib12aa1n02x5               g060(.a(new_n155), .b(new_n149), .c(new_n151), .out0(new_n156));
  xorb03aa1n02x5               g061(.a(new_n156), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  inv000aa1d42x5               g062(.a(\a[14] ), .o1(new_n158));
  inv000aa1d42x5               g063(.a(\a[13] ), .o1(new_n159));
  inv000aa1d42x5               g064(.a(\b[12] ), .o1(new_n160));
  oaoi03aa1n02x5               g065(.a(new_n159), .b(new_n160), .c(new_n156), .o1(new_n161));
  xorb03aa1n02x5               g066(.a(new_n161), .b(\b[13] ), .c(new_n158), .out0(\s[14] ));
  and002aa1n06x5               g067(.a(\b[13] ), .b(\a[14] ), .o(new_n163));
  inv000aa1d42x5               g068(.a(new_n163), .o1(new_n164));
  nor002aa1d24x5               g069(.a(\b[14] ), .b(\a[15] ), .o1(new_n165));
  nand42aa1n04x5               g070(.a(\b[14] ), .b(\a[15] ), .o1(new_n166));
  nanb02aa1n02x5               g071(.a(new_n165), .b(new_n166), .out0(new_n167));
  nanp02aa1n02x5               g072(.a(new_n130), .b(new_n151), .o1(new_n168));
  xnrc02aa1n12x5               g073(.a(\b[12] ), .b(\a[13] ), .out0(new_n169));
  aboi22aa1n03x5               g074(.a(\b[13] ), .b(new_n158), .c(new_n159), .d(new_n160), .out0(new_n170));
  aoai13aa1n04x5               g075(.a(new_n170), .b(new_n169), .c(new_n168), .d(new_n155), .o1(new_n171));
  xnbna2aa1n03x5               g076(.a(new_n167), .b(new_n171), .c(new_n164), .out0(\s[15] ));
  inv000aa1d42x5               g077(.a(new_n165), .o1(new_n173));
  nona22aa1n02x4               g078(.a(new_n171), .b(new_n167), .c(new_n163), .out0(new_n174));
  nor022aa1n06x5               g079(.a(\b[15] ), .b(\a[16] ), .o1(new_n175));
  nand42aa1n03x5               g080(.a(\b[15] ), .b(\a[16] ), .o1(new_n176));
  norb02aa1n02x5               g081(.a(new_n176), .b(new_n175), .out0(new_n177));
  aobi12aa1n02x5               g082(.a(new_n177), .b(new_n174), .c(new_n173), .out0(new_n178));
  aoi113aa1n02x5               g083(.a(new_n165), .b(new_n177), .c(new_n171), .d(new_n166), .e(new_n164), .o1(new_n179));
  norp02aa1n02x5               g084(.a(new_n178), .b(new_n179), .o1(\s[16] ));
  nor043aa1n02x5               g085(.a(new_n139), .b(new_n143), .c(new_n97), .o1(new_n181));
  xnrc02aa1n02x5               g086(.a(\b[13] ), .b(\a[14] ), .out0(new_n182));
  nona23aa1n09x5               g087(.a(new_n176), .b(new_n166), .c(new_n165), .d(new_n175), .out0(new_n183));
  nor043aa1n06x5               g088(.a(new_n183), .b(new_n182), .c(new_n169), .o1(new_n184));
  nand23aa1n03x5               g089(.a(new_n184), .b(new_n181), .c(new_n150), .o1(new_n185));
  aoi112aa1n02x5               g090(.a(\b[14] ), .b(\a[15] ), .c(\a[16] ), .d(\b[15] ), .o1(new_n186));
  nano23aa1n03x7               g091(.a(new_n165), .b(new_n175), .c(new_n176), .d(new_n166), .out0(new_n187));
  nona22aa1n02x5               g092(.a(new_n187), .b(new_n170), .c(new_n163), .out0(new_n188));
  nona22aa1n02x4               g093(.a(new_n188), .b(new_n186), .c(new_n175), .out0(new_n189));
  aoi012aa1d18x5               g094(.a(new_n189), .b(new_n154), .c(new_n184), .o1(new_n190));
  oaih12aa1n12x5               g095(.a(new_n190), .b(new_n149), .c(new_n185), .o1(new_n191));
  xorb03aa1n02x5               g096(.a(new_n191), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  nor002aa1d32x5               g097(.a(\b[16] ), .b(\a[17] ), .o1(new_n193));
  nand22aa1n03x5               g098(.a(\b[16] ), .b(\a[17] ), .o1(new_n194));
  tech160nm_fioai012aa1n05x5   g099(.a(new_n194), .b(new_n191), .c(new_n193), .o1(new_n195));
  xnrb03aa1n02x5               g100(.a(new_n195), .b(\b[17] ), .c(\a[18] ), .out0(\s[18] ));
  nor042aa1d18x5               g101(.a(\b[18] ), .b(\a[19] ), .o1(new_n197));
  nanp02aa1n02x5               g102(.a(\b[18] ), .b(\a[19] ), .o1(new_n198));
  nanb02aa1n02x5               g103(.a(new_n197), .b(new_n198), .out0(new_n199));
  inv000aa1d42x5               g104(.a(new_n199), .o1(new_n200));
  nor022aa1n04x5               g105(.a(\b[17] ), .b(\a[18] ), .o1(new_n201));
  nand22aa1n03x5               g106(.a(\b[17] ), .b(\a[18] ), .o1(new_n202));
  nano23aa1n06x5               g107(.a(new_n193), .b(new_n201), .c(new_n202), .d(new_n194), .out0(new_n203));
  aoi012aa1n02x7               g108(.a(new_n201), .b(new_n193), .c(new_n202), .o1(new_n204));
  inv020aa1n04x5               g109(.a(new_n204), .o1(new_n205));
  aoai13aa1n06x5               g110(.a(new_n200), .b(new_n205), .c(new_n191), .d(new_n203), .o1(new_n206));
  aoi112aa1n02x5               g111(.a(new_n200), .b(new_n205), .c(new_n191), .d(new_n203), .o1(new_n207));
  norb02aa1n02x5               g112(.a(new_n206), .b(new_n207), .out0(\s[19] ));
  xnrc02aa1n02x5               g113(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  inv000aa1d42x5               g114(.a(new_n197), .o1(new_n210));
  nor002aa1n03x5               g115(.a(\b[19] ), .b(\a[20] ), .o1(new_n211));
  nand02aa1n02x5               g116(.a(\b[19] ), .b(\a[20] ), .o1(new_n212));
  nanb02aa1n02x5               g117(.a(new_n211), .b(new_n212), .out0(new_n213));
  xobna2aa1n03x5               g118(.a(new_n213), .b(new_n206), .c(new_n210), .out0(\s[20] ));
  nano23aa1n06x5               g119(.a(new_n197), .b(new_n211), .c(new_n212), .d(new_n198), .out0(new_n215));
  nand22aa1n03x5               g120(.a(new_n215), .b(new_n203), .o1(new_n216));
  oaoi13aa1n12x5               g121(.a(new_n216), .b(new_n190), .c(new_n149), .d(new_n185), .o1(new_n217));
  nona23aa1n02x4               g122(.a(new_n212), .b(new_n198), .c(new_n197), .d(new_n211), .out0(new_n218));
  aoi012aa1n02x7               g123(.a(new_n211), .b(new_n197), .c(new_n212), .o1(new_n219));
  oai012aa1n02x5               g124(.a(new_n219), .b(new_n218), .c(new_n204), .o1(new_n220));
  xnrc02aa1n02x5               g125(.a(\b[20] ), .b(\a[21] ), .out0(new_n221));
  oabi12aa1n06x5               g126(.a(new_n221), .b(new_n217), .c(new_n220), .out0(new_n222));
  norb03aa1n02x5               g127(.a(new_n221), .b(new_n217), .c(new_n220), .out0(new_n223));
  norb02aa1n02x5               g128(.a(new_n222), .b(new_n223), .out0(\s[21] ));
  nor042aa1n04x5               g129(.a(\b[20] ), .b(\a[21] ), .o1(new_n225));
  inv000aa1n02x5               g130(.a(new_n225), .o1(new_n226));
  xorc02aa1n02x5               g131(.a(\a[22] ), .b(\b[21] ), .out0(new_n227));
  aobi12aa1n06x5               g132(.a(new_n227), .b(new_n222), .c(new_n226), .out0(new_n228));
  nona22aa1n02x5               g133(.a(new_n222), .b(new_n227), .c(new_n225), .out0(new_n229));
  norb02aa1n03x4               g134(.a(new_n229), .b(new_n228), .out0(\s[22] ));
  inv000aa1d42x5               g135(.a(new_n216), .o1(new_n231));
  inv000aa1d42x5               g136(.a(\a[21] ), .o1(new_n232));
  inv020aa1n04x5               g137(.a(\a[22] ), .o1(new_n233));
  xroi22aa1d06x4               g138(.a(new_n232), .b(\b[20] ), .c(new_n233), .d(\b[21] ), .out0(new_n234));
  nand23aa1n03x5               g139(.a(new_n191), .b(new_n231), .c(new_n234), .o1(new_n235));
  inv020aa1n02x5               g140(.a(new_n219), .o1(new_n236));
  aoai13aa1n06x5               g141(.a(new_n234), .b(new_n236), .c(new_n215), .d(new_n205), .o1(new_n237));
  oao003aa1n09x5               g142(.a(\a[22] ), .b(\b[21] ), .c(new_n226), .carry(new_n238));
  nanp02aa1n02x5               g143(.a(new_n237), .b(new_n238), .o1(new_n239));
  xnrc02aa1n12x5               g144(.a(\b[22] ), .b(\a[23] ), .out0(new_n240));
  aoib12aa1n06x5               g145(.a(new_n240), .b(new_n235), .c(new_n239), .out0(new_n241));
  inv000aa1d42x5               g146(.a(new_n240), .o1(new_n242));
  aoi113aa1n02x5               g147(.a(new_n239), .b(new_n242), .c(new_n191), .d(new_n231), .e(new_n234), .o1(new_n243));
  norp02aa1n02x5               g148(.a(new_n241), .b(new_n243), .o1(\s[23] ));
  nor042aa1n03x5               g149(.a(\b[22] ), .b(\a[23] ), .o1(new_n245));
  inv000aa1d42x5               g150(.a(new_n245), .o1(new_n246));
  aoai13aa1n02x5               g151(.a(new_n242), .b(new_n239), .c(new_n217), .d(new_n234), .o1(new_n247));
  xnrc02aa1n03x5               g152(.a(\b[23] ), .b(\a[24] ), .out0(new_n248));
  tech160nm_fiaoi012aa1n02p5x5 g153(.a(new_n248), .b(new_n247), .c(new_n246), .o1(new_n249));
  nano22aa1n03x7               g154(.a(new_n241), .b(new_n246), .c(new_n248), .out0(new_n250));
  norp02aa1n02x5               g155(.a(new_n249), .b(new_n250), .o1(\s[24] ));
  nor042aa1n09x5               g156(.a(new_n248), .b(new_n240), .o1(new_n252));
  nand02aa1n04x5               g157(.a(new_n234), .b(new_n252), .o1(new_n253));
  nona22aa1n03x5               g158(.a(new_n191), .b(new_n216), .c(new_n253), .out0(new_n254));
  inv000aa1d42x5               g159(.a(new_n252), .o1(new_n255));
  oao003aa1n02x5               g160(.a(\a[24] ), .b(\b[23] ), .c(new_n246), .carry(new_n256));
  aoai13aa1n12x5               g161(.a(new_n256), .b(new_n255), .c(new_n237), .d(new_n238), .o1(new_n257));
  inv000aa1d42x5               g162(.a(new_n257), .o1(new_n258));
  xnrc02aa1n12x5               g163(.a(\b[24] ), .b(\a[25] ), .out0(new_n259));
  inv000aa1d42x5               g164(.a(new_n259), .o1(new_n260));
  xnbna2aa1n03x5               g165(.a(new_n260), .b(new_n254), .c(new_n258), .out0(\s[25] ));
  nor042aa1n03x5               g166(.a(\b[24] ), .b(\a[25] ), .o1(new_n262));
  inv000aa1d42x5               g167(.a(new_n262), .o1(new_n263));
  inv000aa1d42x5               g168(.a(new_n253), .o1(new_n264));
  aoai13aa1n03x5               g169(.a(new_n260), .b(new_n257), .c(new_n217), .d(new_n264), .o1(new_n265));
  xnrc02aa1n02x5               g170(.a(\b[25] ), .b(\a[26] ), .out0(new_n266));
  aoi012aa1n03x5               g171(.a(new_n266), .b(new_n265), .c(new_n263), .o1(new_n267));
  aoi012aa1n03x5               g172(.a(new_n259), .b(new_n254), .c(new_n258), .o1(new_n268));
  nano22aa1n03x7               g173(.a(new_n268), .b(new_n263), .c(new_n266), .out0(new_n269));
  norp02aa1n03x5               g174(.a(new_n267), .b(new_n269), .o1(\s[26] ));
  nor042aa1n03x5               g175(.a(new_n266), .b(new_n259), .o1(new_n271));
  inv000aa1d42x5               g176(.a(new_n271), .o1(new_n272));
  nona32aa1n09x5               g177(.a(new_n191), .b(new_n272), .c(new_n253), .d(new_n216), .out0(new_n273));
  oao003aa1n02x5               g178(.a(\a[26] ), .b(\b[25] ), .c(new_n263), .carry(new_n274));
  aobi12aa1n12x5               g179(.a(new_n274), .b(new_n257), .c(new_n271), .out0(new_n275));
  norp02aa1n02x5               g180(.a(\b[26] ), .b(\a[27] ), .o1(new_n276));
  nanp02aa1n02x5               g181(.a(\b[26] ), .b(\a[27] ), .o1(new_n277));
  norb02aa1n02x5               g182(.a(new_n277), .b(new_n276), .out0(new_n278));
  xnbna2aa1n03x5               g183(.a(new_n278), .b(new_n275), .c(new_n273), .out0(\s[27] ));
  inv000aa1n06x5               g184(.a(new_n276), .o1(new_n280));
  xnrc02aa1n02x5               g185(.a(\b[27] ), .b(\a[28] ), .out0(new_n281));
  and003aa1n02x5               g186(.a(new_n234), .b(new_n271), .c(new_n252), .o(new_n282));
  inv000aa1d42x5               g187(.a(new_n238), .o1(new_n283));
  aoai13aa1n02x5               g188(.a(new_n252), .b(new_n283), .c(new_n220), .d(new_n234), .o1(new_n284));
  aoai13aa1n04x5               g189(.a(new_n274), .b(new_n272), .c(new_n284), .d(new_n256), .o1(new_n285));
  aoai13aa1n02x5               g190(.a(new_n277), .b(new_n285), .c(new_n217), .d(new_n282), .o1(new_n286));
  tech160nm_fiaoi012aa1n02p5x5 g191(.a(new_n281), .b(new_n286), .c(new_n280), .o1(new_n287));
  aoi022aa1n03x5               g192(.a(new_n275), .b(new_n273), .c(\a[27] ), .d(\b[26] ), .o1(new_n288));
  nano22aa1n03x5               g193(.a(new_n288), .b(new_n280), .c(new_n281), .out0(new_n289));
  norp02aa1n03x5               g194(.a(new_n287), .b(new_n289), .o1(\s[28] ));
  nano22aa1n02x4               g195(.a(new_n281), .b(new_n280), .c(new_n277), .out0(new_n291));
  aoai13aa1n02x5               g196(.a(new_n291), .b(new_n285), .c(new_n217), .d(new_n282), .o1(new_n292));
  oao003aa1n02x5               g197(.a(\a[28] ), .b(\b[27] ), .c(new_n280), .carry(new_n293));
  xnrc02aa1n02x5               g198(.a(\b[28] ), .b(\a[29] ), .out0(new_n294));
  tech160nm_fiaoi012aa1n02p5x5 g199(.a(new_n294), .b(new_n292), .c(new_n293), .o1(new_n295));
  aobi12aa1n06x5               g200(.a(new_n291), .b(new_n275), .c(new_n273), .out0(new_n296));
  nano22aa1n03x5               g201(.a(new_n296), .b(new_n293), .c(new_n294), .out0(new_n297));
  norp02aa1n03x5               g202(.a(new_n295), .b(new_n297), .o1(\s[29] ));
  xorb03aa1n02x5               g203(.a(new_n108), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g204(.a(new_n278), .b(new_n294), .c(new_n281), .out0(new_n300));
  aoai13aa1n02x5               g205(.a(new_n300), .b(new_n285), .c(new_n217), .d(new_n282), .o1(new_n301));
  oao003aa1n02x5               g206(.a(\a[29] ), .b(\b[28] ), .c(new_n293), .carry(new_n302));
  xnrc02aa1n02x5               g207(.a(\b[29] ), .b(\a[30] ), .out0(new_n303));
  tech160nm_fiaoi012aa1n02p5x5 g208(.a(new_n303), .b(new_n301), .c(new_n302), .o1(new_n304));
  aobi12aa1n06x5               g209(.a(new_n300), .b(new_n275), .c(new_n273), .out0(new_n305));
  nano22aa1n03x5               g210(.a(new_n305), .b(new_n302), .c(new_n303), .out0(new_n306));
  norp02aa1n03x5               g211(.a(new_n304), .b(new_n306), .o1(\s[30] ));
  norb03aa1n02x5               g212(.a(new_n291), .b(new_n303), .c(new_n294), .out0(new_n308));
  aoai13aa1n02x5               g213(.a(new_n308), .b(new_n285), .c(new_n217), .d(new_n282), .o1(new_n309));
  oao003aa1n02x5               g214(.a(\a[30] ), .b(\b[29] ), .c(new_n302), .carry(new_n310));
  xnrc02aa1n02x5               g215(.a(\b[30] ), .b(\a[31] ), .out0(new_n311));
  tech160nm_fiaoi012aa1n02p5x5 g216(.a(new_n311), .b(new_n309), .c(new_n310), .o1(new_n312));
  aobi12aa1n06x5               g217(.a(new_n308), .b(new_n275), .c(new_n273), .out0(new_n313));
  nano22aa1n03x5               g218(.a(new_n313), .b(new_n310), .c(new_n311), .out0(new_n314));
  norp02aa1n03x5               g219(.a(new_n312), .b(new_n314), .o1(\s[31] ));
  xobna2aa1n03x5               g220(.a(new_n106), .b(new_n109), .c(new_n107), .out0(\s[3] ));
  nanb02aa1n02x5               g221(.a(\b[3] ), .b(new_n110), .out0(new_n317));
  nanp02aa1n02x5               g222(.a(new_n112), .b(new_n119), .o1(new_n318));
  nanp02aa1n02x5               g223(.a(new_n109), .b(new_n107), .o1(new_n319));
  aboi22aa1n03x5               g224(.a(new_n106), .b(new_n319), .c(new_n317), .d(new_n119), .out0(new_n320));
  aboi22aa1n03x5               g225(.a(new_n318), .b(new_n317), .c(new_n320), .d(new_n104), .out0(\s[4] ));
  xnbna2aa1n03x5               g226(.a(new_n113), .b(new_n112), .c(new_n119), .out0(\s[5] ));
  oaoi03aa1n02x5               g227(.a(\a[5] ), .b(\b[4] ), .c(new_n318), .o1(new_n323));
  xorb03aa1n02x5               g228(.a(new_n323), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  norb02aa1n02x5               g229(.a(new_n115), .b(new_n114), .out0(new_n325));
  oaoi13aa1n02x5               g230(.a(new_n325), .b(new_n116), .c(new_n323), .d(new_n122), .o1(new_n326));
  oai112aa1n02x5               g231(.a(new_n116), .b(new_n325), .c(new_n323), .d(new_n122), .o1(new_n327));
  norb02aa1n02x5               g232(.a(new_n327), .b(new_n326), .out0(\s[7] ));
  orn002aa1n02x5               g233(.a(\a[7] ), .b(\b[6] ), .o(new_n329));
  xobna2aa1n03x5               g234(.a(new_n125), .b(new_n327), .c(new_n329), .out0(\s[8] ));
  xnbna2aa1n03x5               g235(.a(new_n150), .b(new_n124), .c(new_n129), .out0(\s[9] ));
endmodule


