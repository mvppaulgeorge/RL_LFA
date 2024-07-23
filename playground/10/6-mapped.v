// Benchmark "adder" written by ABC on Wed Jul 17 17:05:28 2024

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
    new_n125, new_n126, new_n127, new_n128, new_n129, new_n131, new_n132,
    new_n133, new_n135, new_n136, new_n137, new_n138, new_n139, new_n140,
    new_n141, new_n142, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n157, new_n158, new_n159, new_n160, new_n161, new_n162, new_n163,
    new_n165, new_n166, new_n167, new_n168, new_n169, new_n171, new_n172,
    new_n173, new_n174, new_n175, new_n176, new_n177, new_n178, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n193, new_n194, new_n195,
    new_n196, new_n198, new_n199, new_n200, new_n201, new_n202, new_n203,
    new_n204, new_n205, new_n208, new_n209, new_n210, new_n211, new_n212,
    new_n213, new_n215, new_n216, new_n217, new_n218, new_n219, new_n220,
    new_n221, new_n222, new_n223, new_n225, new_n226, new_n227, new_n228,
    new_n229, new_n231, new_n232, new_n233, new_n234, new_n235, new_n236,
    new_n237, new_n238, new_n239, new_n241, new_n242, new_n243, new_n244,
    new_n245, new_n246, new_n248, new_n249, new_n250, new_n251, new_n252,
    new_n253, new_n254, new_n255, new_n256, new_n257, new_n258, new_n260,
    new_n261, new_n262, new_n263, new_n264, new_n266, new_n267, new_n268,
    new_n269, new_n270, new_n271, new_n272, new_n273, new_n274, new_n275,
    new_n276, new_n277, new_n279, new_n280, new_n281, new_n282, new_n283,
    new_n284, new_n285, new_n286, new_n287, new_n288, new_n290, new_n291,
    new_n292, new_n293, new_n294, new_n295, new_n296, new_n299, new_n300,
    new_n301, new_n302, new_n303, new_n304, new_n305, new_n307, new_n308,
    new_n309, new_n310, new_n311, new_n312, new_n313, new_n316, new_n319,
    new_n321, new_n322, new_n323, new_n325;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor002aa1d24x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  inv000aa1d42x5               g002(.a(new_n97), .o1(new_n98));
  nor022aa1n04x5               g003(.a(\b[7] ), .b(\a[8] ), .o1(new_n99));
  tech160nm_finand02aa1n03p5x5 g004(.a(\b[7] ), .b(\a[8] ), .o1(new_n100));
  nor022aa1n16x5               g005(.a(\b[6] ), .b(\a[7] ), .o1(new_n101));
  tech160nm_fiaoi012aa1n03p5x5 g006(.a(new_n99), .b(new_n101), .c(new_n100), .o1(new_n102));
  and002aa1n18x5               g007(.a(\b[5] ), .b(\a[6] ), .o(new_n103));
  nor002aa1n03x5               g008(.a(\b[4] ), .b(\a[5] ), .o1(new_n104));
  oab012aa1n02x5               g009(.a(new_n104), .b(\a[6] ), .c(\b[5] ), .out0(new_n105));
  nanp02aa1n04x5               g010(.a(\b[6] ), .b(\a[7] ), .o1(new_n106));
  nona23aa1n09x5               g011(.a(new_n106), .b(new_n100), .c(new_n99), .d(new_n101), .out0(new_n107));
  oai013aa1n03x5               g012(.a(new_n102), .b(new_n107), .c(new_n103), .d(new_n105), .o1(new_n108));
  nanp02aa1n02x5               g013(.a(\b[1] ), .b(\a[2] ), .o1(new_n109));
  nand02aa1d04x5               g014(.a(\b[0] ), .b(\a[1] ), .o1(new_n110));
  nor042aa1n02x5               g015(.a(\b[1] ), .b(\a[2] ), .o1(new_n111));
  tech160nm_fioai012aa1n05x5   g016(.a(new_n109), .b(new_n111), .c(new_n110), .o1(new_n112));
  nor022aa1n04x5               g017(.a(\b[3] ), .b(\a[4] ), .o1(new_n113));
  nand02aa1n03x5               g018(.a(\b[3] ), .b(\a[4] ), .o1(new_n114));
  nor022aa1n16x5               g019(.a(\b[2] ), .b(\a[3] ), .o1(new_n115));
  nanp02aa1n02x5               g020(.a(\b[2] ), .b(\a[3] ), .o1(new_n116));
  nona23aa1n09x5               g021(.a(new_n116), .b(new_n114), .c(new_n113), .d(new_n115), .out0(new_n117));
  oai012aa1n02x5               g022(.a(new_n114), .b(new_n115), .c(new_n113), .o1(new_n118));
  oaih12aa1n06x5               g023(.a(new_n118), .b(new_n117), .c(new_n112), .o1(new_n119));
  xnrc02aa1n02x5               g024(.a(\b[5] ), .b(\a[6] ), .out0(new_n120));
  nanp02aa1n02x5               g025(.a(\b[4] ), .b(\a[5] ), .o1(new_n121));
  nanb02aa1n02x5               g026(.a(new_n104), .b(new_n121), .out0(new_n122));
  nor043aa1n03x5               g027(.a(new_n107), .b(new_n120), .c(new_n122), .o1(new_n123));
  nand02aa1n06x5               g028(.a(\b[8] ), .b(\a[9] ), .o1(new_n124));
  norb02aa1n02x5               g029(.a(new_n124), .b(new_n97), .out0(new_n125));
  aoai13aa1n02x5               g030(.a(new_n125), .b(new_n108), .c(new_n123), .d(new_n119), .o1(new_n126));
  nor002aa1d24x5               g031(.a(\b[9] ), .b(\a[10] ), .o1(new_n127));
  nand02aa1d24x5               g032(.a(\b[9] ), .b(\a[10] ), .o1(new_n128));
  norb02aa1n02x5               g033(.a(new_n128), .b(new_n127), .out0(new_n129));
  xnbna2aa1n03x5               g034(.a(new_n129), .b(new_n126), .c(new_n98), .out0(\s[10] ));
  inv000aa1d42x5               g035(.a(new_n127), .o1(new_n131));
  inv000aa1d42x5               g036(.a(new_n128), .o1(new_n132));
  aoi013aa1n03x5               g037(.a(new_n132), .b(new_n126), .c(new_n131), .d(new_n98), .o1(new_n133));
  xorb03aa1n02x5               g038(.a(new_n133), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  nor042aa1n06x5               g039(.a(\b[10] ), .b(\a[11] ), .o1(new_n135));
  nand42aa1d28x5               g040(.a(\b[10] ), .b(\a[11] ), .o1(new_n136));
  norb02aa1n02x5               g041(.a(new_n136), .b(new_n135), .out0(new_n137));
  nor042aa1n06x5               g042(.a(\b[11] ), .b(\a[12] ), .o1(new_n138));
  nand42aa1d28x5               g043(.a(\b[11] ), .b(\a[12] ), .o1(new_n139));
  norb02aa1n02x5               g044(.a(new_n139), .b(new_n138), .out0(new_n140));
  aoi112aa1n02x5               g045(.a(new_n140), .b(new_n135), .c(new_n133), .d(new_n137), .o1(new_n141));
  aoai13aa1n02x5               g046(.a(new_n140), .b(new_n135), .c(new_n133), .d(new_n137), .o1(new_n142));
  norb02aa1n02x5               g047(.a(new_n142), .b(new_n141), .out0(\s[12] ));
  nano23aa1d15x5               g048(.a(new_n138), .b(new_n135), .c(new_n139), .d(new_n136), .out0(new_n144));
  nano23aa1d15x5               g049(.a(new_n97), .b(new_n127), .c(new_n128), .d(new_n124), .out0(new_n145));
  nand22aa1n12x5               g050(.a(new_n145), .b(new_n144), .o1(new_n146));
  inv000aa1d42x5               g051(.a(new_n146), .o1(new_n147));
  aoai13aa1n02x5               g052(.a(new_n147), .b(new_n108), .c(new_n123), .d(new_n119), .o1(new_n148));
  oab012aa1n06x5               g053(.a(new_n132), .b(new_n97), .c(new_n127), .out0(new_n149));
  aoi012aa1n02x7               g054(.a(new_n138), .b(new_n135), .c(new_n139), .o1(new_n150));
  inv000aa1n03x5               g055(.a(new_n150), .o1(new_n151));
  tech160nm_fiaoi012aa1n05x5   g056(.a(new_n151), .b(new_n144), .c(new_n149), .o1(new_n152));
  nanp02aa1n02x5               g057(.a(\b[12] ), .b(\a[13] ), .o1(new_n153));
  nor002aa1n02x5               g058(.a(\b[12] ), .b(\a[13] ), .o1(new_n154));
  norb02aa1n03x5               g059(.a(new_n153), .b(new_n154), .out0(new_n155));
  xnbna2aa1n03x5               g060(.a(new_n155), .b(new_n148), .c(new_n152), .out0(\s[13] ));
  inv000aa1d42x5               g061(.a(\a[13] ), .o1(new_n157));
  inv000aa1d42x5               g062(.a(\b[12] ), .o1(new_n158));
  norp03aa1n02x5               g063(.a(new_n107), .b(new_n105), .c(new_n103), .o1(new_n159));
  norb02aa1n02x5               g064(.a(new_n102), .b(new_n159), .out0(new_n160));
  nanp02aa1n02x5               g065(.a(new_n119), .b(new_n123), .o1(new_n161));
  aoai13aa1n02x5               g066(.a(new_n152), .b(new_n146), .c(new_n160), .d(new_n161), .o1(new_n162));
  oaoi03aa1n02x5               g067(.a(new_n157), .b(new_n158), .c(new_n162), .o1(new_n163));
  xnrb03aa1n02x5               g068(.a(new_n163), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  nor022aa1n08x5               g069(.a(\b[13] ), .b(\a[14] ), .o1(new_n165));
  nand42aa1n03x5               g070(.a(\b[13] ), .b(\a[14] ), .o1(new_n166));
  nona23aa1n02x4               g071(.a(new_n153), .b(new_n166), .c(new_n165), .d(new_n154), .out0(new_n167));
  aoai13aa1n04x5               g072(.a(new_n166), .b(new_n165), .c(new_n157), .d(new_n158), .o1(new_n168));
  aoai13aa1n06x5               g073(.a(new_n168), .b(new_n167), .c(new_n148), .d(new_n152), .o1(new_n169));
  xorb03aa1n02x5               g074(.a(new_n169), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  nor042aa1n02x5               g075(.a(\b[14] ), .b(\a[15] ), .o1(new_n171));
  nanp02aa1n04x5               g076(.a(\b[14] ), .b(\a[15] ), .o1(new_n172));
  norb02aa1n02x5               g077(.a(new_n172), .b(new_n171), .out0(new_n173));
  nor042aa1n06x5               g078(.a(\b[15] ), .b(\a[16] ), .o1(new_n174));
  nand42aa1n02x5               g079(.a(\b[15] ), .b(\a[16] ), .o1(new_n175));
  norb02aa1n06x4               g080(.a(new_n175), .b(new_n174), .out0(new_n176));
  aoi112aa1n02x5               g081(.a(new_n176), .b(new_n171), .c(new_n169), .d(new_n173), .o1(new_n177));
  aoai13aa1n02x5               g082(.a(new_n176), .b(new_n171), .c(new_n169), .d(new_n172), .o1(new_n178));
  norb02aa1n03x4               g083(.a(new_n178), .b(new_n177), .out0(\s[16] ));
  norb02aa1n03x5               g084(.a(new_n166), .b(new_n165), .out0(new_n180));
  nano23aa1n06x5               g085(.a(new_n174), .b(new_n171), .c(new_n175), .d(new_n172), .out0(new_n181));
  nano32aa1d12x5               g086(.a(new_n146), .b(new_n181), .c(new_n155), .d(new_n180), .out0(new_n182));
  aoai13aa1n12x5               g087(.a(new_n182), .b(new_n108), .c(new_n123), .d(new_n119), .o1(new_n183));
  inv000aa1d42x5               g088(.a(new_n174), .o1(new_n184));
  aoi112aa1n03x5               g089(.a(\b[14] ), .b(\a[15] ), .c(\a[16] ), .d(\b[15] ), .o1(new_n185));
  inv000aa1d42x5               g090(.a(new_n185), .o1(new_n186));
  nanb03aa1n02x5               g091(.a(new_n168), .b(new_n176), .c(new_n173), .out0(new_n187));
  nanp03aa1n02x5               g092(.a(new_n181), .b(new_n155), .c(new_n180), .o1(new_n188));
  nor002aa1n02x5               g093(.a(new_n152), .b(new_n188), .o1(new_n189));
  nano32aa1n03x7               g094(.a(new_n189), .b(new_n187), .c(new_n186), .d(new_n184), .out0(new_n190));
  nanp02aa1n12x5               g095(.a(new_n190), .b(new_n183), .o1(new_n191));
  xorb03aa1n02x5               g096(.a(new_n191), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv000aa1d42x5               g097(.a(\a[18] ), .o1(new_n193));
  inv000aa1d42x5               g098(.a(\a[17] ), .o1(new_n194));
  inv000aa1d42x5               g099(.a(\b[16] ), .o1(new_n195));
  oaoi03aa1n02x5               g100(.a(new_n194), .b(new_n195), .c(new_n191), .o1(new_n196));
  xorb03aa1n02x5               g101(.a(new_n196), .b(\b[17] ), .c(new_n193), .out0(\s[18] ));
  xroi22aa1d06x4               g102(.a(new_n194), .b(\b[16] ), .c(new_n193), .d(\b[17] ), .out0(new_n198));
  nanp02aa1n02x5               g103(.a(new_n195), .b(new_n194), .o1(new_n199));
  oaoi03aa1n02x5               g104(.a(\a[18] ), .b(\b[17] ), .c(new_n199), .o1(new_n200));
  nand42aa1n04x5               g105(.a(\b[18] ), .b(\a[19] ), .o1(new_n201));
  nor042aa1n06x5               g106(.a(\b[18] ), .b(\a[19] ), .o1(new_n202));
  norb02aa1n02x5               g107(.a(new_n201), .b(new_n202), .out0(new_n203));
  aoai13aa1n06x5               g108(.a(new_n203), .b(new_n200), .c(new_n191), .d(new_n198), .o1(new_n204));
  aoi112aa1n02x5               g109(.a(new_n203), .b(new_n200), .c(new_n191), .d(new_n198), .o1(new_n205));
  norb02aa1n02x5               g110(.a(new_n204), .b(new_n205), .out0(\s[19] ));
  xnrc02aa1n02x5               g111(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor002aa1n06x5               g112(.a(\b[19] ), .b(\a[20] ), .o1(new_n208));
  nand42aa1n06x5               g113(.a(\b[19] ), .b(\a[20] ), .o1(new_n209));
  norb02aa1n02x5               g114(.a(new_n209), .b(new_n208), .out0(new_n210));
  nona22aa1n02x5               g115(.a(new_n204), .b(new_n210), .c(new_n202), .out0(new_n211));
  inv000aa1d42x5               g116(.a(new_n210), .o1(new_n212));
  oaoi13aa1n06x5               g117(.a(new_n212), .b(new_n204), .c(\a[19] ), .d(\b[18] ), .o1(new_n213));
  norb02aa1n03x4               g118(.a(new_n211), .b(new_n213), .out0(\s[20] ));
  nano23aa1n03x5               g119(.a(new_n208), .b(new_n202), .c(new_n209), .d(new_n201), .out0(new_n215));
  nanp02aa1n02x5               g120(.a(new_n198), .b(new_n215), .o1(new_n216));
  oai022aa1n02x5               g121(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n217));
  oaib12aa1n06x5               g122(.a(new_n217), .b(new_n193), .c(\b[17] ), .out0(new_n218));
  nona23aa1n09x5               g123(.a(new_n201), .b(new_n209), .c(new_n208), .d(new_n202), .out0(new_n219));
  aoi012aa1n06x5               g124(.a(new_n208), .b(new_n202), .c(new_n209), .o1(new_n220));
  oai012aa1n18x5               g125(.a(new_n220), .b(new_n219), .c(new_n218), .o1(new_n221));
  inv000aa1d42x5               g126(.a(new_n221), .o1(new_n222));
  aoai13aa1n06x5               g127(.a(new_n222), .b(new_n216), .c(new_n190), .d(new_n183), .o1(new_n223));
  xorb03aa1n02x5               g128(.a(new_n223), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor042aa1n02x5               g129(.a(\b[20] ), .b(\a[21] ), .o1(new_n225));
  xorc02aa1n02x5               g130(.a(\a[21] ), .b(\b[20] ), .out0(new_n226));
  xorc02aa1n02x5               g131(.a(\a[22] ), .b(\b[21] ), .out0(new_n227));
  aoi112aa1n02x5               g132(.a(new_n225), .b(new_n227), .c(new_n223), .d(new_n226), .o1(new_n228));
  aoai13aa1n06x5               g133(.a(new_n227), .b(new_n225), .c(new_n223), .d(new_n226), .o1(new_n229));
  norb02aa1n02x5               g134(.a(new_n229), .b(new_n228), .out0(\s[22] ));
  inv000aa1d42x5               g135(.a(\a[21] ), .o1(new_n231));
  inv000aa1d42x5               g136(.a(\a[22] ), .o1(new_n232));
  xroi22aa1d04x5               g137(.a(new_n231), .b(\b[20] ), .c(new_n232), .d(\b[21] ), .out0(new_n233));
  nanp03aa1n02x5               g138(.a(new_n233), .b(new_n198), .c(new_n215), .o1(new_n234));
  inv000aa1d42x5               g139(.a(\b[21] ), .o1(new_n235));
  oaoi03aa1n12x5               g140(.a(new_n232), .b(new_n235), .c(new_n225), .o1(new_n236));
  inv000aa1d42x5               g141(.a(new_n236), .o1(new_n237));
  aoi012aa1n02x5               g142(.a(new_n237), .b(new_n221), .c(new_n233), .o1(new_n238));
  aoai13aa1n06x5               g143(.a(new_n238), .b(new_n234), .c(new_n190), .d(new_n183), .o1(new_n239));
  xorb03aa1n02x5               g144(.a(new_n239), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  nor042aa1n04x5               g145(.a(\b[22] ), .b(\a[23] ), .o1(new_n241));
  and002aa1n12x5               g146(.a(\b[22] ), .b(\a[23] ), .o(new_n242));
  inv000aa1d42x5               g147(.a(new_n242), .o1(new_n243));
  tech160nm_fixorc02aa1n05x5   g148(.a(\a[24] ), .b(\b[23] ), .out0(new_n244));
  aoi112aa1n02x5               g149(.a(new_n241), .b(new_n244), .c(new_n239), .d(new_n243), .o1(new_n245));
  aoai13aa1n03x5               g150(.a(new_n244), .b(new_n241), .c(new_n239), .d(new_n243), .o1(new_n246));
  norb02aa1n02x5               g151(.a(new_n246), .b(new_n245), .out0(\s[24] ));
  nano22aa1n02x4               g152(.a(new_n241), .b(new_n244), .c(new_n243), .out0(new_n248));
  inv000aa1n02x5               g153(.a(new_n248), .o1(new_n249));
  nor002aa1n02x5               g154(.a(new_n234), .b(new_n249), .o1(new_n250));
  inv000aa1n02x5               g155(.a(new_n220), .o1(new_n251));
  aoai13aa1n03x5               g156(.a(new_n233), .b(new_n251), .c(new_n215), .d(new_n200), .o1(new_n252));
  inv000aa1d42x5               g157(.a(new_n241), .o1(new_n253));
  oao003aa1n02x5               g158(.a(\a[24] ), .b(\b[23] ), .c(new_n253), .carry(new_n254));
  aoai13aa1n06x5               g159(.a(new_n254), .b(new_n249), .c(new_n252), .d(new_n236), .o1(new_n255));
  tech160nm_fixorc02aa1n05x5   g160(.a(\a[25] ), .b(\b[24] ), .out0(new_n256));
  aoai13aa1n06x5               g161(.a(new_n256), .b(new_n255), .c(new_n191), .d(new_n250), .o1(new_n257));
  aoi112aa1n02x5               g162(.a(new_n256), .b(new_n255), .c(new_n191), .d(new_n250), .o1(new_n258));
  norb02aa1n02x5               g163(.a(new_n257), .b(new_n258), .out0(\s[25] ));
  nor042aa1n03x5               g164(.a(\b[24] ), .b(\a[25] ), .o1(new_n260));
  xorc02aa1n02x5               g165(.a(\a[26] ), .b(\b[25] ), .out0(new_n261));
  nona22aa1n02x5               g166(.a(new_n257), .b(new_n261), .c(new_n260), .out0(new_n262));
  inv000aa1d42x5               g167(.a(new_n260), .o1(new_n263));
  aobi12aa1n06x5               g168(.a(new_n261), .b(new_n257), .c(new_n263), .out0(new_n264));
  norb02aa1n03x4               g169(.a(new_n262), .b(new_n264), .out0(\s[26] ));
  nand02aa1n02x5               g170(.a(new_n160), .b(new_n161), .o1(new_n266));
  nano22aa1n03x7               g171(.a(new_n167), .b(new_n173), .c(new_n176), .out0(new_n267));
  aoai13aa1n04x5               g172(.a(new_n267), .b(new_n151), .c(new_n144), .d(new_n149), .o1(new_n268));
  nona23aa1n02x5               g173(.a(new_n268), .b(new_n187), .c(new_n174), .d(new_n185), .out0(new_n269));
  and002aa1n06x5               g174(.a(new_n261), .b(new_n256), .o(new_n270));
  nano22aa1n03x7               g175(.a(new_n234), .b(new_n248), .c(new_n270), .out0(new_n271));
  aoai13aa1n06x5               g176(.a(new_n271), .b(new_n269), .c(new_n266), .d(new_n182), .o1(new_n272));
  oao003aa1n02x5               g177(.a(\a[26] ), .b(\b[25] ), .c(new_n263), .carry(new_n273));
  aobi12aa1n06x5               g178(.a(new_n273), .b(new_n255), .c(new_n270), .out0(new_n274));
  norp02aa1n02x5               g179(.a(\b[26] ), .b(\a[27] ), .o1(new_n275));
  nanp02aa1n02x5               g180(.a(\b[26] ), .b(\a[27] ), .o1(new_n276));
  norb02aa1n02x5               g181(.a(new_n276), .b(new_n275), .out0(new_n277));
  xnbna2aa1n03x5               g182(.a(new_n277), .b(new_n274), .c(new_n272), .out0(\s[27] ));
  inv000aa1n06x5               g183(.a(new_n275), .o1(new_n279));
  xnrc02aa1n02x5               g184(.a(\b[27] ), .b(\a[28] ), .out0(new_n280));
  aobi12aa1n02x7               g185(.a(new_n276), .b(new_n274), .c(new_n272), .out0(new_n281));
  nano22aa1n03x5               g186(.a(new_n281), .b(new_n279), .c(new_n280), .out0(new_n282));
  aobi12aa1n06x5               g187(.a(new_n271), .b(new_n190), .c(new_n183), .out0(new_n283));
  aoai13aa1n03x5               g188(.a(new_n248), .b(new_n237), .c(new_n221), .d(new_n233), .o1(new_n284));
  inv000aa1d42x5               g189(.a(new_n270), .o1(new_n285));
  aoai13aa1n06x5               g190(.a(new_n273), .b(new_n285), .c(new_n284), .d(new_n254), .o1(new_n286));
  tech160nm_fioai012aa1n04x5   g191(.a(new_n276), .b(new_n286), .c(new_n283), .o1(new_n287));
  aoi012aa1n03x5               g192(.a(new_n280), .b(new_n287), .c(new_n279), .o1(new_n288));
  norp02aa1n03x5               g193(.a(new_n288), .b(new_n282), .o1(\s[28] ));
  nano22aa1n02x4               g194(.a(new_n280), .b(new_n279), .c(new_n276), .out0(new_n290));
  oaih12aa1n02x5               g195(.a(new_n290), .b(new_n286), .c(new_n283), .o1(new_n291));
  oao003aa1n02x5               g196(.a(\a[28] ), .b(\b[27] ), .c(new_n279), .carry(new_n292));
  xnrc02aa1n02x5               g197(.a(\b[28] ), .b(\a[29] ), .out0(new_n293));
  tech160nm_fiaoi012aa1n02p5x5 g198(.a(new_n293), .b(new_n291), .c(new_n292), .o1(new_n294));
  aobi12aa1n02x7               g199(.a(new_n290), .b(new_n274), .c(new_n272), .out0(new_n295));
  nano22aa1n03x5               g200(.a(new_n295), .b(new_n292), .c(new_n293), .out0(new_n296));
  norp02aa1n03x5               g201(.a(new_n294), .b(new_n296), .o1(\s[29] ));
  xorb03aa1n02x5               g202(.a(new_n110), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g203(.a(new_n277), .b(new_n293), .c(new_n280), .out0(new_n299));
  oaih12aa1n02x5               g204(.a(new_n299), .b(new_n286), .c(new_n283), .o1(new_n300));
  oao003aa1n03x5               g205(.a(\a[29] ), .b(\b[28] ), .c(new_n292), .carry(new_n301));
  xnrc02aa1n02x5               g206(.a(\b[29] ), .b(\a[30] ), .out0(new_n302));
  tech160nm_fiaoi012aa1n05x5   g207(.a(new_n302), .b(new_n300), .c(new_n301), .o1(new_n303));
  aobi12aa1n06x5               g208(.a(new_n299), .b(new_n274), .c(new_n272), .out0(new_n304));
  nano22aa1n03x5               g209(.a(new_n304), .b(new_n301), .c(new_n302), .out0(new_n305));
  nor002aa1n02x5               g210(.a(new_n303), .b(new_n305), .o1(\s[30] ));
  xnrc02aa1n02x5               g211(.a(\b[30] ), .b(\a[31] ), .out0(new_n307));
  norb03aa1n02x5               g212(.a(new_n290), .b(new_n302), .c(new_n293), .out0(new_n308));
  aobi12aa1n02x7               g213(.a(new_n308), .b(new_n274), .c(new_n272), .out0(new_n309));
  oao003aa1n02x5               g214(.a(\a[30] ), .b(\b[29] ), .c(new_n301), .carry(new_n310));
  nano22aa1n03x5               g215(.a(new_n309), .b(new_n307), .c(new_n310), .out0(new_n311));
  oaih12aa1n02x5               g216(.a(new_n308), .b(new_n286), .c(new_n283), .o1(new_n312));
  tech160nm_fiaoi012aa1n02p5x5 g217(.a(new_n307), .b(new_n312), .c(new_n310), .o1(new_n313));
  norp02aa1n03x5               g218(.a(new_n313), .b(new_n311), .o1(\s[31] ));
  xnrb03aa1n02x5               g219(.a(new_n112), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g220(.a(\a[3] ), .b(\b[2] ), .c(new_n112), .o1(new_n316));
  xorb03aa1n02x5               g221(.a(new_n316), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g222(.a(new_n119), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  oai012aa1n02x5               g223(.a(new_n121), .b(new_n119), .c(new_n104), .o1(new_n319));
  xnrb03aa1n02x5               g224(.a(new_n319), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  inv000aa1d42x5               g225(.a(new_n103), .o1(new_n321));
  nanb02aa1n02x5               g226(.a(new_n101), .b(new_n106), .out0(new_n322));
  nanb02aa1n02x5               g227(.a(new_n120), .b(new_n319), .out0(new_n323));
  xnbna2aa1n03x5               g228(.a(new_n322), .b(new_n323), .c(new_n321), .out0(\s[7] ));
  aoi013aa1n02x4               g229(.a(new_n101), .b(new_n323), .c(new_n106), .d(new_n321), .o1(new_n325));
  xnrb03aa1n02x5               g230(.a(new_n325), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xnbna2aa1n03x5               g231(.a(new_n125), .b(new_n160), .c(new_n161), .out0(\s[9] ));
endmodule


