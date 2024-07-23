// Benchmark "adder" written by ABC on Wed Jul 17 14:11:33 2024

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
    new_n132, new_n133, new_n134, new_n135, new_n137, new_n138, new_n139,
    new_n141, new_n142, new_n143, new_n144, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n155, new_n156,
    new_n157, new_n159, new_n160, new_n161, new_n162, new_n163, new_n164,
    new_n165, new_n166, new_n167, new_n169, new_n170, new_n171, new_n172,
    new_n173, new_n175, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n184, new_n186, new_n187, new_n188, new_n189,
    new_n190, new_n191, new_n192, new_n193, new_n194, new_n195, new_n198,
    new_n199, new_n200, new_n201, new_n202, new_n203, new_n205, new_n206,
    new_n207, new_n208, new_n209, new_n210, new_n212, new_n213, new_n214,
    new_n215, new_n216, new_n218, new_n219, new_n220, new_n221, new_n222,
    new_n223, new_n224, new_n225, new_n226, new_n227, new_n228, new_n229,
    new_n230, new_n232, new_n233, new_n234, new_n235, new_n236, new_n237,
    new_n239, new_n240, new_n241, new_n242, new_n243, new_n244, new_n245,
    new_n246, new_n248, new_n249, new_n250, new_n251, new_n252, new_n253,
    new_n254, new_n256, new_n257, new_n258, new_n259, new_n260, new_n261,
    new_n263, new_n264, new_n265, new_n266, new_n267, new_n268, new_n269,
    new_n270, new_n272, new_n273, new_n274, new_n275, new_n276, new_n277,
    new_n278, new_n279, new_n281, new_n283, new_n284, new_n285, new_n286,
    new_n287, new_n288, new_n289, new_n290, new_n292, new_n293, new_n294,
    new_n295, new_n296, new_n297, new_n298, new_n299, new_n301, new_n303,
    new_n304, new_n305, new_n306, new_n309, new_n310, new_n312, new_n313,
    new_n314, new_n316;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  xorc02aa1n12x5               g001(.a(\a[10] ), .b(\b[9] ), .out0(new_n97));
  inv000aa1d42x5               g002(.a(new_n97), .o1(new_n98));
  norp02aa1n02x5               g003(.a(\b[8] ), .b(\a[9] ), .o1(new_n99));
  nand42aa1n02x5               g004(.a(\b[8] ), .b(\a[9] ), .o1(new_n100));
  norp02aa1n04x5               g005(.a(\b[1] ), .b(\a[2] ), .o1(new_n101));
  aoi022aa1d24x5               g006(.a(\b[1] ), .b(\a[2] ), .c(\a[1] ), .d(\b[0] ), .o1(new_n102));
  nor042aa1n04x5               g007(.a(\b[2] ), .b(\a[3] ), .o1(new_n103));
  nand42aa1n06x5               g008(.a(\b[2] ), .b(\a[3] ), .o1(new_n104));
  norb02aa1n06x5               g009(.a(new_n104), .b(new_n103), .out0(new_n105));
  oai022aa1d18x5               g010(.a(\a[3] ), .b(\b[2] ), .c(\b[3] ), .d(\a[4] ), .o1(new_n106));
  oaoi13aa1n12x5               g011(.a(new_n106), .b(new_n105), .c(new_n102), .d(new_n101), .o1(new_n107));
  nand42aa1n03x5               g012(.a(\b[4] ), .b(\a[5] ), .o1(new_n108));
  norp02aa1n02x5               g013(.a(\b[4] ), .b(\a[5] ), .o1(new_n109));
  norb02aa1n02x7               g014(.a(new_n108), .b(new_n109), .out0(new_n110));
  nor042aa1n02x5               g015(.a(\b[6] ), .b(\a[7] ), .o1(new_n111));
  nanp02aa1n02x5               g016(.a(\b[6] ), .b(\a[7] ), .o1(new_n112));
  nand42aa1n03x5               g017(.a(\b[5] ), .b(\a[6] ), .o1(new_n113));
  nano22aa1n03x7               g018(.a(new_n111), .b(new_n112), .c(new_n113), .out0(new_n114));
  nand42aa1n06x5               g019(.a(\b[3] ), .b(\a[4] ), .o1(new_n115));
  oai012aa1n02x5               g020(.a(new_n115), .b(\b[7] ), .c(\a[8] ), .o1(new_n116));
  tech160nm_finand02aa1n05x5   g021(.a(\b[7] ), .b(\a[8] ), .o1(new_n117));
  oai012aa1n02x5               g022(.a(new_n117), .b(\b[5] ), .c(\a[6] ), .o1(new_n118));
  norp02aa1n02x5               g023(.a(new_n118), .b(new_n116), .o1(new_n119));
  nanp03aa1n02x5               g024(.a(new_n119), .b(new_n114), .c(new_n110), .o1(new_n120));
  oai022aa1n02x5               g025(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n121));
  norp02aa1n02x5               g026(.a(\b[7] ), .b(\a[8] ), .o1(new_n122));
  norb02aa1n02x5               g027(.a(new_n117), .b(new_n122), .out0(new_n123));
  inv000aa1n02x5               g028(.a(new_n111), .o1(new_n124));
  oaoi03aa1n02x5               g029(.a(\a[8] ), .b(\b[7] ), .c(new_n124), .o1(new_n125));
  aoi013aa1n02x5               g030(.a(new_n125), .b(new_n114), .c(new_n121), .d(new_n123), .o1(new_n126));
  oai012aa1n12x5               g031(.a(new_n126), .b(new_n120), .c(new_n107), .o1(new_n127));
  aoai13aa1n02x5               g032(.a(new_n98), .b(new_n99), .c(new_n127), .d(new_n100), .o1(new_n128));
  inv000aa1d42x5               g033(.a(\a[10] ), .o1(new_n129));
  inv000aa1d42x5               g034(.a(\b[9] ), .o1(new_n130));
  nand42aa1n04x5               g035(.a(new_n130), .b(new_n129), .o1(new_n131));
  nand42aa1n08x5               g036(.a(\b[9] ), .b(\a[10] ), .o1(new_n132));
  oai112aa1n06x5               g037(.a(new_n131), .b(new_n132), .c(\b[8] ), .d(\a[9] ), .o1(new_n133));
  inv000aa1d42x5               g038(.a(new_n133), .o1(new_n134));
  aob012aa1n02x5               g039(.a(new_n134), .b(new_n127), .c(new_n100), .out0(new_n135));
  nanp02aa1n02x5               g040(.a(new_n128), .b(new_n135), .o1(\s[10] ));
  nand42aa1n02x5               g041(.a(\b[10] ), .b(\a[11] ), .o1(new_n137));
  norp02aa1n02x5               g042(.a(\b[10] ), .b(\a[11] ), .o1(new_n138));
  norb02aa1n02x7               g043(.a(new_n137), .b(new_n138), .out0(new_n139));
  xobna2aa1n03x5               g044(.a(new_n139), .b(new_n135), .c(new_n132), .out0(\s[11] ));
  norp02aa1n02x5               g045(.a(\b[11] ), .b(\a[12] ), .o1(new_n141));
  nand42aa1n03x5               g046(.a(\b[11] ), .b(\a[12] ), .o1(new_n142));
  norb02aa1n02x5               g047(.a(new_n142), .b(new_n141), .out0(new_n143));
  aoi013aa1n02x4               g048(.a(new_n138), .b(new_n135), .c(new_n137), .d(new_n132), .o1(new_n144));
  xnrc02aa1n02x5               g049(.a(new_n144), .b(new_n143), .out0(\s[12] ));
  nano23aa1n02x4               g050(.a(new_n99), .b(new_n141), .c(new_n142), .d(new_n100), .out0(new_n146));
  and003aa1n02x5               g051(.a(new_n146), .b(new_n139), .c(new_n97), .o(new_n147));
  nand22aa1n03x5               g052(.a(new_n127), .b(new_n147), .o1(new_n148));
  nano22aa1n02x4               g053(.a(new_n138), .b(new_n132), .c(new_n137), .out0(new_n149));
  nand03aa1n04x5               g054(.a(new_n149), .b(new_n133), .c(new_n143), .o1(new_n150));
  aoi012aa1n02x5               g055(.a(new_n141), .b(new_n138), .c(new_n142), .o1(new_n151));
  and002aa1n02x5               g056(.a(new_n150), .b(new_n151), .o(new_n152));
  nand02aa1d06x5               g057(.a(new_n148), .b(new_n152), .o1(new_n153));
  xorb03aa1n02x5               g058(.a(new_n153), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  nor042aa1n03x5               g059(.a(\b[12] ), .b(\a[13] ), .o1(new_n155));
  nanp02aa1n02x5               g060(.a(\b[12] ), .b(\a[13] ), .o1(new_n156));
  tech160nm_fiaoi012aa1n05x5   g061(.a(new_n155), .b(new_n153), .c(new_n156), .o1(new_n157));
  xnrb03aa1n03x5               g062(.a(new_n157), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  nor002aa1n02x5               g063(.a(\b[13] ), .b(\a[14] ), .o1(new_n159));
  nanp02aa1n02x5               g064(.a(\b[13] ), .b(\a[14] ), .o1(new_n160));
  nano23aa1n06x5               g065(.a(new_n155), .b(new_n159), .c(new_n160), .d(new_n156), .out0(new_n161));
  tech160nm_fiao0012aa1n05x5   g066(.a(new_n159), .b(new_n155), .c(new_n160), .o(new_n162));
  nor002aa1n02x5               g067(.a(\b[14] ), .b(\a[15] ), .o1(new_n163));
  nanp02aa1n02x5               g068(.a(\b[14] ), .b(\a[15] ), .o1(new_n164));
  norb02aa1n02x5               g069(.a(new_n164), .b(new_n163), .out0(new_n165));
  aoai13aa1n12x5               g070(.a(new_n165), .b(new_n162), .c(new_n153), .d(new_n161), .o1(new_n166));
  aoi112aa1n02x5               g071(.a(new_n165), .b(new_n162), .c(new_n153), .d(new_n161), .o1(new_n167));
  norb02aa1n02x5               g072(.a(new_n166), .b(new_n167), .out0(\s[15] ));
  nor002aa1n02x5               g073(.a(\b[15] ), .b(\a[16] ), .o1(new_n169));
  nanp02aa1n02x5               g074(.a(\b[15] ), .b(\a[16] ), .o1(new_n170));
  nanb02aa1n02x5               g075(.a(new_n169), .b(new_n170), .out0(new_n171));
  oaoi13aa1n06x5               g076(.a(new_n171), .b(new_n166), .c(\a[15] ), .d(\b[14] ), .o1(new_n172));
  oai112aa1n03x5               g077(.a(new_n166), .b(new_n171), .c(\b[14] ), .d(\a[15] ), .o1(new_n173));
  norb02aa1n02x7               g078(.a(new_n173), .b(new_n172), .out0(\s[16] ));
  inv000aa1d42x5               g079(.a(\a[17] ), .o1(new_n175));
  nano23aa1n02x5               g080(.a(new_n163), .b(new_n169), .c(new_n170), .d(new_n164), .out0(new_n176));
  nanp02aa1n02x5               g081(.a(new_n176), .b(new_n161), .o1(new_n177));
  nano32aa1n03x7               g082(.a(new_n177), .b(new_n146), .c(new_n139), .d(new_n97), .out0(new_n178));
  tech160nm_fiao0012aa1n02p5x5 g083(.a(new_n169), .b(new_n163), .c(new_n170), .o(new_n179));
  aoi012aa1n02x5               g084(.a(new_n179), .b(new_n176), .c(new_n162), .o1(new_n180));
  aoai13aa1n06x5               g085(.a(new_n180), .b(new_n177), .c(new_n150), .d(new_n151), .o1(new_n181));
  aoi012aa1n06x5               g086(.a(new_n181), .b(new_n127), .c(new_n178), .o1(new_n182));
  xorb03aa1n02x5               g087(.a(new_n182), .b(\b[16] ), .c(new_n175), .out0(\s[17] ));
  oaoi03aa1n02x5               g088(.a(\a[17] ), .b(\b[16] ), .c(new_n182), .o1(new_n184));
  xorb03aa1n02x5               g089(.a(new_n184), .b(\b[17] ), .c(\a[18] ), .out0(\s[18] ));
  inv000aa1d42x5               g090(.a(\a[18] ), .o1(new_n186));
  xroi22aa1d04x5               g091(.a(new_n175), .b(\b[16] ), .c(new_n186), .d(\b[17] ), .out0(new_n187));
  aoai13aa1n06x5               g092(.a(new_n187), .b(new_n181), .c(new_n127), .d(new_n178), .o1(new_n188));
  inv000aa1d42x5               g093(.a(\b[17] ), .o1(new_n189));
  norp02aa1n02x5               g094(.a(\b[16] ), .b(\a[17] ), .o1(new_n190));
  oao003aa1n02x5               g095(.a(new_n186), .b(new_n189), .c(new_n190), .carry(new_n191));
  inv000aa1d42x5               g096(.a(new_n191), .o1(new_n192));
  nor042aa1n06x5               g097(.a(\b[18] ), .b(\a[19] ), .o1(new_n193));
  nand02aa1d04x5               g098(.a(\b[18] ), .b(\a[19] ), .o1(new_n194));
  norb02aa1n09x5               g099(.a(new_n194), .b(new_n193), .out0(new_n195));
  xnbna2aa1n03x5               g100(.a(new_n195), .b(new_n188), .c(new_n192), .out0(\s[19] ));
  xnrc02aa1n02x5               g101(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  aobi12aa1n02x5               g102(.a(new_n195), .b(new_n188), .c(new_n192), .out0(new_n198));
  nor042aa1n04x5               g103(.a(\b[19] ), .b(\a[20] ), .o1(new_n199));
  nand02aa1d06x5               g104(.a(\b[19] ), .b(\a[20] ), .o1(new_n200));
  norb02aa1n06x5               g105(.a(new_n200), .b(new_n199), .out0(new_n201));
  oaih12aa1n02x5               g106(.a(new_n201), .b(new_n198), .c(new_n193), .o1(new_n202));
  norp03aa1n02x5               g107(.a(new_n198), .b(new_n201), .c(new_n193), .o1(new_n203));
  norb02aa1n02x5               g108(.a(new_n202), .b(new_n203), .out0(\s[20] ));
  nano23aa1n02x4               g109(.a(new_n193), .b(new_n199), .c(new_n200), .d(new_n194), .out0(new_n205));
  and002aa1n02x5               g110(.a(new_n187), .b(new_n205), .o(new_n206));
  aoai13aa1n06x5               g111(.a(new_n206), .b(new_n181), .c(new_n127), .d(new_n178), .o1(new_n207));
  tech160nm_fiaoi012aa1n05x5   g112(.a(new_n199), .b(new_n193), .c(new_n200), .o1(new_n208));
  aobi12aa1n02x5               g113(.a(new_n208), .b(new_n205), .c(new_n191), .out0(new_n209));
  xorc02aa1n12x5               g114(.a(\a[21] ), .b(\b[20] ), .out0(new_n210));
  xnbna2aa1n03x5               g115(.a(new_n210), .b(new_n207), .c(new_n209), .out0(\s[21] ));
  norp02aa1n02x5               g116(.a(\b[20] ), .b(\a[21] ), .o1(new_n212));
  aobi12aa1n03x5               g117(.a(new_n210), .b(new_n207), .c(new_n209), .out0(new_n213));
  xnrc02aa1n12x5               g118(.a(\b[21] ), .b(\a[22] ), .out0(new_n214));
  oab012aa1n03x5               g119(.a(new_n214), .b(new_n213), .c(new_n212), .out0(new_n215));
  norb03aa1n02x5               g120(.a(new_n214), .b(new_n213), .c(new_n212), .out0(new_n216));
  norp02aa1n03x5               g121(.a(new_n215), .b(new_n216), .o1(\s[22] ));
  nanb02aa1d24x5               g122(.a(new_n214), .b(new_n210), .out0(new_n218));
  nor002aa1n02x5               g123(.a(\b[17] ), .b(\a[18] ), .o1(new_n219));
  aoi112aa1n09x5               g124(.a(\b[16] ), .b(\a[17] ), .c(\a[18] ), .d(\b[17] ), .o1(new_n220));
  oai112aa1n06x5               g125(.a(new_n195), .b(new_n201), .c(new_n220), .d(new_n219), .o1(new_n221));
  aoi112aa1n02x5               g126(.a(\b[20] ), .b(\a[21] ), .c(\a[22] ), .d(\b[21] ), .o1(new_n222));
  oab012aa1n04x5               g127(.a(new_n222), .b(\a[22] ), .c(\b[21] ), .out0(new_n223));
  aoai13aa1n12x5               g128(.a(new_n223), .b(new_n218), .c(new_n221), .d(new_n208), .o1(new_n224));
  inv000aa1d42x5               g129(.a(new_n224), .o1(new_n225));
  xnrc02aa1n03x5               g130(.a(\b[22] ), .b(\a[23] ), .out0(new_n226));
  oaoi13aa1n06x5               g131(.a(new_n226), .b(new_n225), .c(new_n207), .d(new_n218), .o1(new_n227));
  inv000aa1d42x5               g132(.a(new_n218), .o1(new_n228));
  nano22aa1n03x7               g133(.a(new_n182), .b(new_n206), .c(new_n228), .out0(new_n229));
  nano22aa1n02x4               g134(.a(new_n229), .b(new_n225), .c(new_n226), .out0(new_n230));
  norp02aa1n02x5               g135(.a(new_n227), .b(new_n230), .o1(\s[23] ));
  nor042aa1n06x5               g136(.a(\b[22] ), .b(\a[23] ), .o1(new_n232));
  inv000aa1d42x5               g137(.a(new_n232), .o1(new_n233));
  oabi12aa1n06x5               g138(.a(new_n226), .b(new_n229), .c(new_n224), .out0(new_n234));
  tech160nm_fixorc02aa1n03p5x5 g139(.a(\a[24] ), .b(\b[23] ), .out0(new_n235));
  aobi12aa1n03x5               g140(.a(new_n235), .b(new_n234), .c(new_n233), .out0(new_n236));
  norp03aa1n03x5               g141(.a(new_n227), .b(new_n235), .c(new_n232), .o1(new_n237));
  norp02aa1n03x5               g142(.a(new_n236), .b(new_n237), .o1(\s[24] ));
  norb02aa1n03x5               g143(.a(new_n235), .b(new_n226), .out0(new_n239));
  oaoi03aa1n02x5               g144(.a(\a[24] ), .b(\b[23] ), .c(new_n233), .o1(new_n240));
  aoi012aa1d24x5               g145(.a(new_n240), .b(new_n224), .c(new_n239), .o1(new_n241));
  nona23aa1n02x4               g146(.a(new_n210), .b(new_n235), .c(new_n226), .d(new_n214), .out0(new_n242));
  xnrc02aa1n02x5               g147(.a(\b[24] ), .b(\a[25] ), .out0(new_n243));
  oaoi13aa1n09x5               g148(.a(new_n243), .b(new_n241), .c(new_n207), .d(new_n242), .o1(new_n244));
  nano32aa1n03x7               g149(.a(new_n182), .b(new_n239), .c(new_n206), .d(new_n228), .out0(new_n245));
  nano22aa1n02x4               g150(.a(new_n245), .b(new_n241), .c(new_n243), .out0(new_n246));
  norp02aa1n02x5               g151(.a(new_n244), .b(new_n246), .o1(\s[25] ));
  norp02aa1n02x5               g152(.a(\b[24] ), .b(\a[25] ), .o1(new_n248));
  inv000aa1n02x5               g153(.a(new_n248), .o1(new_n249));
  inv000aa1d42x5               g154(.a(new_n241), .o1(new_n250));
  oabi12aa1n02x5               g155(.a(new_n243), .b(new_n245), .c(new_n250), .out0(new_n251));
  xorc02aa1n02x5               g156(.a(\a[26] ), .b(\b[25] ), .out0(new_n252));
  aobi12aa1n02x7               g157(.a(new_n252), .b(new_n251), .c(new_n249), .out0(new_n253));
  nor043aa1n03x5               g158(.a(new_n244), .b(new_n252), .c(new_n248), .o1(new_n254));
  nor042aa1n03x5               g159(.a(new_n253), .b(new_n254), .o1(\s[26] ));
  norb02aa1n02x5               g160(.a(new_n252), .b(new_n243), .out0(new_n256));
  nano32aa1n03x7               g161(.a(new_n242), .b(new_n256), .c(new_n187), .d(new_n205), .out0(new_n257));
  aoai13aa1n12x5               g162(.a(new_n257), .b(new_n181), .c(new_n127), .d(new_n178), .o1(new_n258));
  aoai13aa1n09x5               g163(.a(new_n256), .b(new_n240), .c(new_n224), .d(new_n239), .o1(new_n259));
  oao003aa1n02x5               g164(.a(\a[26] ), .b(\b[25] ), .c(new_n249), .carry(new_n260));
  nand23aa1n09x5               g165(.a(new_n259), .b(new_n258), .c(new_n260), .o1(new_n261));
  xorb03aa1n03x5               g166(.a(new_n261), .b(\b[26] ), .c(\a[27] ), .out0(\s[27] ));
  nor042aa1d18x5               g167(.a(\b[26] ), .b(\a[27] ), .o1(new_n263));
  inv000aa1n06x5               g168(.a(new_n263), .o1(new_n264));
  nanp02aa1n02x5               g169(.a(\b[26] ), .b(\a[27] ), .o1(new_n265));
  nanp02aa1n06x5               g170(.a(new_n261), .b(new_n265), .o1(new_n266));
  xorc02aa1n12x5               g171(.a(\a[28] ), .b(\b[27] ), .out0(new_n267));
  inv000aa1d42x5               g172(.a(new_n267), .o1(new_n268));
  aoi012aa1n03x5               g173(.a(new_n268), .b(new_n266), .c(new_n264), .o1(new_n269));
  aoi112aa1n03x4               g174(.a(new_n263), .b(new_n267), .c(new_n261), .d(new_n265), .o1(new_n270));
  nor002aa1n02x5               g175(.a(new_n269), .b(new_n270), .o1(\s[28] ));
  nano22aa1n02x5               g176(.a(new_n268), .b(new_n264), .c(new_n265), .out0(new_n272));
  tech160nm_finand02aa1n05x5   g177(.a(new_n261), .b(new_n272), .o1(new_n273));
  oao003aa1n12x5               g178(.a(\a[28] ), .b(\b[27] ), .c(new_n264), .carry(new_n274));
  xorc02aa1n12x5               g179(.a(\a[29] ), .b(\b[28] ), .out0(new_n275));
  inv000aa1d42x5               g180(.a(new_n275), .o1(new_n276));
  tech160nm_fiaoi012aa1n05x5   g181(.a(new_n276), .b(new_n273), .c(new_n274), .o1(new_n277));
  inv000aa1d42x5               g182(.a(new_n274), .o1(new_n278));
  aoi112aa1n02x7               g183(.a(new_n275), .b(new_n278), .c(new_n261), .d(new_n272), .o1(new_n279));
  nor002aa1n02x5               g184(.a(new_n277), .b(new_n279), .o1(\s[29] ));
  nanp02aa1n02x5               g185(.a(\b[0] ), .b(\a[1] ), .o1(new_n281));
  xorb03aa1n02x5               g186(.a(new_n281), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano32aa1n02x4               g187(.a(new_n276), .b(new_n267), .c(new_n265), .d(new_n264), .out0(new_n283));
  tech160nm_finand02aa1n05x5   g188(.a(new_n261), .b(new_n283), .o1(new_n284));
  oao003aa1n09x5               g189(.a(\a[29] ), .b(\b[28] ), .c(new_n274), .carry(new_n285));
  tech160nm_fixorc02aa1n03p5x5 g190(.a(\a[30] ), .b(\b[29] ), .out0(new_n286));
  inv000aa1d42x5               g191(.a(new_n286), .o1(new_n287));
  tech160nm_fiaoi012aa1n05x5   g192(.a(new_n287), .b(new_n284), .c(new_n285), .o1(new_n288));
  inv000aa1d42x5               g193(.a(new_n285), .o1(new_n289));
  aoi112aa1n02x7               g194(.a(new_n286), .b(new_n289), .c(new_n261), .d(new_n283), .o1(new_n290));
  norp02aa1n03x5               g195(.a(new_n288), .b(new_n290), .o1(\s[30] ));
  xnrc02aa1n02x5               g196(.a(\b[30] ), .b(\a[31] ), .out0(new_n292));
  and003aa1n02x5               g197(.a(new_n272), .b(new_n286), .c(new_n275), .o(new_n293));
  nand22aa1n03x5               g198(.a(new_n261), .b(new_n293), .o1(new_n294));
  oaoi03aa1n12x5               g199(.a(\a[30] ), .b(\b[29] ), .c(new_n285), .o1(new_n295));
  inv000aa1d42x5               g200(.a(new_n295), .o1(new_n296));
  tech160nm_fiaoi012aa1n05x5   g201(.a(new_n292), .b(new_n294), .c(new_n296), .o1(new_n297));
  inv000aa1d42x5               g202(.a(new_n292), .o1(new_n298));
  aoi112aa1n03x5               g203(.a(new_n298), .b(new_n295), .c(new_n261), .d(new_n293), .o1(new_n299));
  nor042aa1n03x5               g204(.a(new_n297), .b(new_n299), .o1(\s[31] ));
  norp02aa1n02x5               g205(.a(new_n102), .b(new_n101), .o1(new_n301));
  xnrc02aa1n02x5               g206(.a(new_n301), .b(new_n105), .out0(\s[3] ));
  orn002aa1n02x5               g207(.a(\a[4] ), .b(\b[3] ), .o(new_n303));
  inv000aa1d42x5               g208(.a(new_n107), .o1(new_n304));
  tech160nm_fiao0012aa1n02p5x5 g209(.a(new_n103), .b(new_n303), .c(new_n115), .o(new_n305));
  aoib12aa1n02x5               g210(.a(new_n305), .b(new_n105), .c(new_n301), .out0(new_n306));
  aoi013aa1n02x4               g211(.a(new_n306), .b(new_n304), .c(new_n115), .d(new_n303), .o1(\s[4] ));
  xobna2aa1n03x5               g212(.a(new_n110), .b(new_n304), .c(new_n115), .out0(\s[5] ));
  xorc02aa1n02x5               g213(.a(\a[6] ), .b(\b[5] ), .out0(new_n309));
  aoai13aa1n02x5               g214(.a(new_n108), .b(new_n109), .c(new_n304), .d(new_n115), .o1(new_n310));
  xnrc02aa1n02x5               g215(.a(new_n310), .b(new_n309), .out0(\s[6] ));
  aobi12aa1n02x5               g216(.a(new_n114), .b(new_n310), .c(new_n309), .out0(new_n312));
  nanp02aa1n02x5               g217(.a(new_n310), .b(new_n309), .o1(new_n313));
  aoi022aa1n02x5               g218(.a(new_n313), .b(new_n113), .c(new_n124), .d(new_n112), .o1(new_n314));
  norp02aa1n02x5               g219(.a(new_n314), .b(new_n312), .o1(\s[7] ));
  aoi012aa1n02x5               g220(.a(new_n111), .b(new_n313), .c(new_n114), .o1(new_n316));
  xnrc02aa1n02x5               g221(.a(new_n316), .b(new_n123), .out0(\s[8] ));
  xorb03aa1n02x5               g222(.a(new_n127), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


