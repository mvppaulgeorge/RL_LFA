// Benchmark "adder" written by ABC on Wed Jul 17 14:41:43 2024

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
    new_n126, new_n127, new_n128, new_n129, new_n130, new_n131, new_n132,
    new_n133, new_n134, new_n135, new_n137, new_n138, new_n139, new_n140,
    new_n141, new_n142, new_n143, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n153, new_n154, new_n155, new_n157,
    new_n158, new_n159, new_n160, new_n161, new_n162, new_n163, new_n164,
    new_n165, new_n166, new_n167, new_n169, new_n170, new_n171, new_n172,
    new_n173, new_n174, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n183, new_n184, new_n185, new_n187, new_n188, new_n189,
    new_n190, new_n191, new_n192, new_n193, new_n194, new_n195, new_n196,
    new_n199, new_n200, new_n201, new_n202, new_n203, new_n204, new_n206,
    new_n207, new_n208, new_n209, new_n210, new_n211, new_n212, new_n213,
    new_n214, new_n215, new_n217, new_n218, new_n219, new_n220, new_n221,
    new_n223, new_n224, new_n225, new_n226, new_n227, new_n228, new_n229,
    new_n230, new_n231, new_n232, new_n234, new_n235, new_n236, new_n237,
    new_n238, new_n240, new_n241, new_n242, new_n243, new_n244, new_n245,
    new_n246, new_n247, new_n248, new_n249, new_n250, new_n251, new_n253,
    new_n254, new_n255, new_n256, new_n257, new_n259, new_n260, new_n261,
    new_n262, new_n263, new_n264, new_n265, new_n267, new_n268, new_n269,
    new_n270, new_n271, new_n272, new_n273, new_n274, new_n275, new_n277,
    new_n278, new_n279, new_n280, new_n281, new_n282, new_n283, new_n286,
    new_n287, new_n288, new_n289, new_n290, new_n291, new_n292, new_n294,
    new_n295, new_n296, new_n297, new_n298, new_n299, new_n300, new_n303,
    new_n306, new_n308, new_n310;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nand22aa1n02x5               g001(.a(\b[7] ), .b(\a[8] ), .o1(new_n97));
  norp02aa1n24x5               g002(.a(\b[7] ), .b(\a[8] ), .o1(new_n98));
  norp02aa1n12x5               g003(.a(\b[6] ), .b(\a[7] ), .o1(new_n99));
  nanp02aa1n02x5               g004(.a(\b[6] ), .b(\a[7] ), .o1(new_n100));
  nona23aa1n06x5               g005(.a(new_n97), .b(new_n100), .c(new_n99), .d(new_n98), .out0(new_n101));
  inv000aa1d42x5               g006(.a(\a[5] ), .o1(new_n102));
  inv000aa1d42x5               g007(.a(\b[4] ), .o1(new_n103));
  nor042aa1n03x5               g008(.a(\b[5] ), .b(\a[6] ), .o1(new_n104));
  nand22aa1n02x5               g009(.a(\b[5] ), .b(\a[6] ), .o1(new_n105));
  aoi013aa1n03x5               g010(.a(new_n104), .b(new_n105), .c(new_n102), .d(new_n103), .o1(new_n106));
  tech160nm_fiaoi012aa1n02p5x5 g011(.a(new_n98), .b(new_n99), .c(new_n97), .o1(new_n107));
  oai012aa1n04x7               g012(.a(new_n107), .b(new_n101), .c(new_n106), .o1(new_n108));
  nor022aa1n04x5               g013(.a(\b[1] ), .b(\a[2] ), .o1(new_n109));
  nand22aa1n06x5               g014(.a(\b[0] ), .b(\a[1] ), .o1(new_n110));
  nand02aa1d04x5               g015(.a(\b[1] ), .b(\a[2] ), .o1(new_n111));
  aoi012aa1n12x5               g016(.a(new_n109), .b(new_n110), .c(new_n111), .o1(new_n112));
  nor002aa1n12x5               g017(.a(\b[3] ), .b(\a[4] ), .o1(new_n113));
  nand42aa1n04x5               g018(.a(\b[3] ), .b(\a[4] ), .o1(new_n114));
  nor022aa1n16x5               g019(.a(\b[2] ), .b(\a[3] ), .o1(new_n115));
  nand02aa1n03x5               g020(.a(\b[2] ), .b(\a[3] ), .o1(new_n116));
  nona23aa1n09x5               g021(.a(new_n116), .b(new_n114), .c(new_n113), .d(new_n115), .out0(new_n117));
  tech160nm_fioai012aa1n03p5x5 g022(.a(new_n114), .b(new_n115), .c(new_n113), .o1(new_n118));
  oai012aa1n12x5               g023(.a(new_n118), .b(new_n117), .c(new_n112), .o1(new_n119));
  nanb02aa1n02x5               g024(.a(new_n104), .b(new_n105), .out0(new_n120));
  xnrc02aa1n02x5               g025(.a(\b[4] ), .b(\a[5] ), .out0(new_n121));
  nor043aa1n03x5               g026(.a(new_n101), .b(new_n120), .c(new_n121), .o1(new_n122));
  aoi012aa1n06x5               g027(.a(new_n108), .b(new_n119), .c(new_n122), .o1(new_n123));
  oaoi03aa1n02x5               g028(.a(\a[9] ), .b(\b[8] ), .c(new_n123), .o1(new_n124));
  xorb03aa1n02x5               g029(.a(new_n124), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  nor002aa1d32x5               g030(.a(\b[10] ), .b(\a[11] ), .o1(new_n126));
  nand22aa1n12x5               g031(.a(\b[10] ), .b(\a[11] ), .o1(new_n127));
  norb02aa1n02x5               g032(.a(new_n127), .b(new_n126), .out0(new_n128));
  nor002aa1d32x5               g033(.a(\b[8] ), .b(\a[9] ), .o1(new_n129));
  nand02aa1d24x5               g034(.a(\b[8] ), .b(\a[9] ), .o1(new_n130));
  nor002aa1d32x5               g035(.a(\b[9] ), .b(\a[10] ), .o1(new_n131));
  nand02aa1d28x5               g036(.a(\b[9] ), .b(\a[10] ), .o1(new_n132));
  nano23aa1n09x5               g037(.a(new_n129), .b(new_n131), .c(new_n132), .d(new_n130), .out0(new_n133));
  aoai13aa1n02x5               g038(.a(new_n133), .b(new_n108), .c(new_n119), .d(new_n122), .o1(new_n134));
  aoi012aa1d24x5               g039(.a(new_n131), .b(new_n129), .c(new_n132), .o1(new_n135));
  xnbna2aa1n03x5               g040(.a(new_n128), .b(new_n134), .c(new_n135), .out0(\s[11] ));
  inv000aa1n06x5               g041(.a(new_n126), .o1(new_n137));
  inv000aa1n06x5               g042(.a(new_n123), .o1(new_n138));
  inv000aa1d42x5               g043(.a(new_n135), .o1(new_n139));
  aoai13aa1n03x5               g044(.a(new_n128), .b(new_n139), .c(new_n138), .d(new_n133), .o1(new_n140));
  nor002aa1d32x5               g045(.a(\b[11] ), .b(\a[12] ), .o1(new_n141));
  nand02aa1n20x5               g046(.a(\b[11] ), .b(\a[12] ), .o1(new_n142));
  norb02aa1n02x5               g047(.a(new_n142), .b(new_n141), .out0(new_n143));
  xnbna2aa1n03x5               g048(.a(new_n143), .b(new_n140), .c(new_n137), .out0(\s[12] ));
  nano23aa1n03x7               g049(.a(new_n126), .b(new_n141), .c(new_n142), .d(new_n127), .out0(new_n145));
  nand42aa1n02x5               g050(.a(new_n145), .b(new_n133), .o1(new_n146));
  nona23aa1n09x5               g051(.a(new_n142), .b(new_n127), .c(new_n126), .d(new_n141), .out0(new_n147));
  oaoi03aa1n02x5               g052(.a(\a[12] ), .b(\b[11] ), .c(new_n137), .o1(new_n148));
  inv040aa1n02x5               g053(.a(new_n148), .o1(new_n149));
  oai012aa1n06x5               g054(.a(new_n149), .b(new_n147), .c(new_n135), .o1(new_n150));
  oabi12aa1n12x5               g055(.a(new_n150), .b(new_n123), .c(new_n146), .out0(new_n151));
  xorb03aa1n02x5               g056(.a(new_n151), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  norp02aa1n06x5               g057(.a(\b[12] ), .b(\a[13] ), .o1(new_n153));
  nanp02aa1n03x5               g058(.a(\b[12] ), .b(\a[13] ), .o1(new_n154));
  aoi012aa1n03x5               g059(.a(new_n153), .b(new_n151), .c(new_n154), .o1(new_n155));
  xnrb03aa1n03x5               g060(.a(new_n155), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  nor002aa1n12x5               g061(.a(\b[14] ), .b(\a[15] ), .o1(new_n157));
  nanp02aa1n02x5               g062(.a(\b[14] ), .b(\a[15] ), .o1(new_n158));
  nanb02aa1n02x5               g063(.a(new_n157), .b(new_n158), .out0(new_n159));
  inv000aa1d42x5               g064(.a(new_n159), .o1(new_n160));
  nor042aa1n02x5               g065(.a(\b[13] ), .b(\a[14] ), .o1(new_n161));
  nand42aa1n02x5               g066(.a(\b[13] ), .b(\a[14] ), .o1(new_n162));
  nano23aa1n06x5               g067(.a(new_n153), .b(new_n161), .c(new_n162), .d(new_n154), .out0(new_n163));
  aoi012aa1n02x5               g068(.a(new_n161), .b(new_n153), .c(new_n162), .o1(new_n164));
  inv000aa1n02x5               g069(.a(new_n164), .o1(new_n165));
  aoai13aa1n06x5               g070(.a(new_n160), .b(new_n165), .c(new_n151), .d(new_n163), .o1(new_n166));
  aoi112aa1n02x5               g071(.a(new_n160), .b(new_n165), .c(new_n151), .d(new_n163), .o1(new_n167));
  norb02aa1n02x7               g072(.a(new_n166), .b(new_n167), .out0(\s[15] ));
  inv000aa1d42x5               g073(.a(new_n157), .o1(new_n169));
  nor042aa1n02x5               g074(.a(\b[15] ), .b(\a[16] ), .o1(new_n170));
  nand02aa1n03x5               g075(.a(\b[15] ), .b(\a[16] ), .o1(new_n171));
  nanb02aa1n02x5               g076(.a(new_n170), .b(new_n171), .out0(new_n172));
  nand43aa1n02x5               g077(.a(new_n166), .b(new_n169), .c(new_n172), .o1(new_n173));
  tech160nm_fiaoi012aa1n03p5x5 g078(.a(new_n172), .b(new_n166), .c(new_n169), .o1(new_n174));
  norb02aa1n03x4               g079(.a(new_n173), .b(new_n174), .out0(\s[16] ));
  nano23aa1n06x5               g080(.a(new_n157), .b(new_n170), .c(new_n171), .d(new_n158), .out0(new_n176));
  nano22aa1n03x7               g081(.a(new_n146), .b(new_n163), .c(new_n176), .out0(new_n177));
  aoai13aa1n06x5               g082(.a(new_n177), .b(new_n108), .c(new_n119), .d(new_n122), .o1(new_n178));
  aoai13aa1n12x5               g083(.a(new_n176), .b(new_n165), .c(new_n150), .d(new_n163), .o1(new_n179));
  tech160nm_fioai012aa1n05x5   g084(.a(new_n171), .b(new_n170), .c(new_n157), .o1(new_n180));
  nand23aa1d12x5               g085(.a(new_n178), .b(new_n179), .c(new_n180), .o1(new_n181));
  xorb03aa1n03x5               g086(.a(new_n181), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  nor042aa1n06x5               g087(.a(\b[16] ), .b(\a[17] ), .o1(new_n183));
  nand42aa1n10x5               g088(.a(\b[16] ), .b(\a[17] ), .o1(new_n184));
  tech160nm_fiaoi012aa1n05x5   g089(.a(new_n183), .b(new_n181), .c(new_n184), .o1(new_n185));
  xnrb03aa1n03x5               g090(.a(new_n185), .b(\b[17] ), .c(\a[18] ), .out0(\s[18] ));
  nor002aa1n04x5               g091(.a(\b[17] ), .b(\a[18] ), .o1(new_n187));
  nand02aa1n10x5               g092(.a(\b[17] ), .b(\a[18] ), .o1(new_n188));
  nano23aa1n09x5               g093(.a(new_n183), .b(new_n187), .c(new_n188), .d(new_n184), .out0(new_n189));
  aoi012aa1n06x5               g094(.a(new_n187), .b(new_n183), .c(new_n188), .o1(new_n190));
  inv030aa1n03x5               g095(.a(new_n190), .o1(new_n191));
  nor042aa1n09x5               g096(.a(\b[18] ), .b(\a[19] ), .o1(new_n192));
  nand02aa1d06x5               g097(.a(\b[18] ), .b(\a[19] ), .o1(new_n193));
  norb02aa1n02x5               g098(.a(new_n193), .b(new_n192), .out0(new_n194));
  aoai13aa1n06x5               g099(.a(new_n194), .b(new_n191), .c(new_n181), .d(new_n189), .o1(new_n195));
  aoi112aa1n02x5               g100(.a(new_n194), .b(new_n191), .c(new_n181), .d(new_n189), .o1(new_n196));
  norb02aa1n02x7               g101(.a(new_n195), .b(new_n196), .out0(\s[19] ));
  xnrc02aa1n02x5               g102(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  norp02aa1n06x5               g103(.a(\b[19] ), .b(\a[20] ), .o1(new_n199));
  nand22aa1n04x5               g104(.a(\b[19] ), .b(\a[20] ), .o1(new_n200));
  norb02aa1n02x5               g105(.a(new_n200), .b(new_n199), .out0(new_n201));
  nona22aa1n02x5               g106(.a(new_n195), .b(new_n201), .c(new_n192), .out0(new_n202));
  inv000aa1n06x5               g107(.a(new_n192), .o1(new_n203));
  aobi12aa1n06x5               g108(.a(new_n201), .b(new_n195), .c(new_n203), .out0(new_n204));
  norb02aa1n03x4               g109(.a(new_n202), .b(new_n204), .out0(\s[20] ));
  nano23aa1n06x5               g110(.a(new_n192), .b(new_n199), .c(new_n200), .d(new_n193), .out0(new_n206));
  nanp02aa1n04x5               g111(.a(new_n206), .b(new_n189), .o1(new_n207));
  inv000aa1d42x5               g112(.a(new_n207), .o1(new_n208));
  nona23aa1n03x5               g113(.a(new_n200), .b(new_n193), .c(new_n192), .d(new_n199), .out0(new_n209));
  oaoi03aa1n03x5               g114(.a(\a[20] ), .b(\b[19] ), .c(new_n203), .o1(new_n210));
  inv030aa1n02x5               g115(.a(new_n210), .o1(new_n211));
  oaih12aa1n06x5               g116(.a(new_n211), .b(new_n209), .c(new_n190), .o1(new_n212));
  xorc02aa1n02x5               g117(.a(\a[21] ), .b(\b[20] ), .out0(new_n213));
  aoai13aa1n06x5               g118(.a(new_n213), .b(new_n212), .c(new_n181), .d(new_n208), .o1(new_n214));
  aoi112aa1n02x5               g119(.a(new_n213), .b(new_n212), .c(new_n181), .d(new_n208), .o1(new_n215));
  norb02aa1n02x7               g120(.a(new_n214), .b(new_n215), .out0(\s[21] ));
  nor042aa1n03x5               g121(.a(\b[20] ), .b(\a[21] ), .o1(new_n217));
  xorc02aa1n02x5               g122(.a(\a[22] ), .b(\b[21] ), .out0(new_n218));
  nona22aa1n06x5               g123(.a(new_n214), .b(new_n218), .c(new_n217), .out0(new_n219));
  inv040aa1n03x5               g124(.a(new_n217), .o1(new_n220));
  aobi12aa1n06x5               g125(.a(new_n218), .b(new_n214), .c(new_n220), .out0(new_n221));
  norb02aa1n03x4               g126(.a(new_n219), .b(new_n221), .out0(\s[22] ));
  nano22aa1n02x5               g127(.a(new_n207), .b(new_n213), .c(new_n218), .out0(new_n223));
  inv000aa1d42x5               g128(.a(\a[21] ), .o1(new_n224));
  inv000aa1d42x5               g129(.a(\a[22] ), .o1(new_n225));
  xroi22aa1d06x4               g130(.a(new_n224), .b(\b[20] ), .c(new_n225), .d(\b[21] ), .out0(new_n226));
  oaoi03aa1n12x5               g131(.a(\a[22] ), .b(\b[21] ), .c(new_n220), .o1(new_n227));
  aoi012aa1n02x5               g132(.a(new_n227), .b(new_n212), .c(new_n226), .o1(new_n228));
  inv000aa1n02x5               g133(.a(new_n228), .o1(new_n229));
  tech160nm_fixorc02aa1n05x5   g134(.a(\a[23] ), .b(\b[22] ), .out0(new_n230));
  aoai13aa1n06x5               g135(.a(new_n230), .b(new_n229), .c(new_n181), .d(new_n223), .o1(new_n231));
  aoi112aa1n02x5               g136(.a(new_n230), .b(new_n229), .c(new_n181), .d(new_n223), .o1(new_n232));
  norb02aa1n02x7               g137(.a(new_n231), .b(new_n232), .out0(\s[23] ));
  norp02aa1n02x5               g138(.a(\b[22] ), .b(\a[23] ), .o1(new_n234));
  xorc02aa1n02x5               g139(.a(\a[24] ), .b(\b[23] ), .out0(new_n235));
  nona22aa1n02x5               g140(.a(new_n231), .b(new_n235), .c(new_n234), .out0(new_n236));
  inv000aa1d42x5               g141(.a(new_n234), .o1(new_n237));
  aobi12aa1n06x5               g142(.a(new_n235), .b(new_n231), .c(new_n237), .out0(new_n238));
  norb02aa1n03x4               g143(.a(new_n236), .b(new_n238), .out0(\s[24] ));
  and002aa1n06x5               g144(.a(new_n235), .b(new_n230), .o(new_n240));
  inv000aa1n02x5               g145(.a(new_n240), .o1(new_n241));
  nano32aa1n02x4               g146(.a(new_n241), .b(new_n226), .c(new_n206), .d(new_n189), .out0(new_n242));
  aoai13aa1n03x5               g147(.a(new_n226), .b(new_n210), .c(new_n206), .d(new_n191), .o1(new_n243));
  inv000aa1n02x5               g148(.a(new_n227), .o1(new_n244));
  nanp02aa1n02x5               g149(.a(\b[23] ), .b(\a[24] ), .o1(new_n245));
  oai022aa1n02x5               g150(.a(\a[23] ), .b(\b[22] ), .c(\b[23] ), .d(\a[24] ), .o1(new_n246));
  nanp02aa1n02x5               g151(.a(new_n246), .b(new_n245), .o1(new_n247));
  aoai13aa1n06x5               g152(.a(new_n247), .b(new_n241), .c(new_n243), .d(new_n244), .o1(new_n248));
  xorc02aa1n12x5               g153(.a(\a[25] ), .b(\b[24] ), .out0(new_n249));
  aoai13aa1n06x5               g154(.a(new_n249), .b(new_n248), .c(new_n181), .d(new_n242), .o1(new_n250));
  aoi112aa1n02x5               g155(.a(new_n249), .b(new_n248), .c(new_n181), .d(new_n242), .o1(new_n251));
  norb02aa1n02x7               g156(.a(new_n250), .b(new_n251), .out0(\s[25] ));
  nor042aa1n03x5               g157(.a(\b[24] ), .b(\a[25] ), .o1(new_n253));
  xorc02aa1n02x5               g158(.a(\a[26] ), .b(\b[25] ), .out0(new_n254));
  nona22aa1n02x5               g159(.a(new_n250), .b(new_n254), .c(new_n253), .out0(new_n255));
  inv000aa1d42x5               g160(.a(new_n253), .o1(new_n256));
  aobi12aa1n06x5               g161(.a(new_n254), .b(new_n250), .c(new_n256), .out0(new_n257));
  norb02aa1n03x4               g162(.a(new_n255), .b(new_n257), .out0(\s[26] ));
  and002aa1n02x5               g163(.a(new_n254), .b(new_n249), .o(new_n259));
  inv000aa1n02x5               g164(.a(new_n259), .o1(new_n260));
  nano23aa1n09x5               g165(.a(new_n207), .b(new_n260), .c(new_n240), .d(new_n226), .out0(new_n261));
  nand22aa1n09x5               g166(.a(new_n181), .b(new_n261), .o1(new_n262));
  oao003aa1n02x5               g167(.a(\a[26] ), .b(\b[25] ), .c(new_n256), .carry(new_n263));
  aobi12aa1n06x5               g168(.a(new_n263), .b(new_n248), .c(new_n259), .out0(new_n264));
  xorc02aa1n12x5               g169(.a(\a[27] ), .b(\b[26] ), .out0(new_n265));
  xnbna2aa1n03x5               g170(.a(new_n265), .b(new_n262), .c(new_n264), .out0(\s[27] ));
  norp02aa1n02x5               g171(.a(\b[26] ), .b(\a[27] ), .o1(new_n267));
  inv040aa1n03x5               g172(.a(new_n267), .o1(new_n268));
  aobi12aa1n02x7               g173(.a(new_n265), .b(new_n262), .c(new_n264), .out0(new_n269));
  xnrc02aa1n02x5               g174(.a(\b[27] ), .b(\a[28] ), .out0(new_n270));
  nano22aa1n02x4               g175(.a(new_n269), .b(new_n268), .c(new_n270), .out0(new_n271));
  aoai13aa1n04x5               g176(.a(new_n240), .b(new_n227), .c(new_n212), .d(new_n226), .o1(new_n272));
  aoai13aa1n06x5               g177(.a(new_n263), .b(new_n260), .c(new_n272), .d(new_n247), .o1(new_n273));
  aoai13aa1n03x5               g178(.a(new_n265), .b(new_n273), .c(new_n181), .d(new_n261), .o1(new_n274));
  tech160nm_fiaoi012aa1n02p5x5 g179(.a(new_n270), .b(new_n274), .c(new_n268), .o1(new_n275));
  norp02aa1n03x5               g180(.a(new_n275), .b(new_n271), .o1(\s[28] ));
  norb02aa1n02x5               g181(.a(new_n265), .b(new_n270), .out0(new_n277));
  aobi12aa1n06x5               g182(.a(new_n277), .b(new_n262), .c(new_n264), .out0(new_n278));
  oao003aa1n02x5               g183(.a(\a[28] ), .b(\b[27] ), .c(new_n268), .carry(new_n279));
  xnrc02aa1n02x5               g184(.a(\b[28] ), .b(\a[29] ), .out0(new_n280));
  nano22aa1n02x4               g185(.a(new_n278), .b(new_n279), .c(new_n280), .out0(new_n281));
  aoai13aa1n03x5               g186(.a(new_n277), .b(new_n273), .c(new_n181), .d(new_n261), .o1(new_n282));
  tech160nm_fiaoi012aa1n02p5x5 g187(.a(new_n280), .b(new_n282), .c(new_n279), .o1(new_n283));
  norp02aa1n03x5               g188(.a(new_n283), .b(new_n281), .o1(\s[29] ));
  xorb03aa1n02x5               g189(.a(new_n110), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g190(.a(new_n265), .b(new_n280), .c(new_n270), .out0(new_n286));
  aobi12aa1n06x5               g191(.a(new_n286), .b(new_n262), .c(new_n264), .out0(new_n287));
  oao003aa1n02x5               g192(.a(\a[29] ), .b(\b[28] ), .c(new_n279), .carry(new_n288));
  xnrc02aa1n02x5               g193(.a(\b[29] ), .b(\a[30] ), .out0(new_n289));
  nano22aa1n02x4               g194(.a(new_n287), .b(new_n288), .c(new_n289), .out0(new_n290));
  aoai13aa1n03x5               g195(.a(new_n286), .b(new_n273), .c(new_n181), .d(new_n261), .o1(new_n291));
  tech160nm_fiaoi012aa1n02p5x5 g196(.a(new_n289), .b(new_n291), .c(new_n288), .o1(new_n292));
  norp02aa1n03x5               g197(.a(new_n292), .b(new_n290), .o1(\s[30] ));
  norb02aa1n02x7               g198(.a(new_n286), .b(new_n289), .out0(new_n294));
  aobi12aa1n02x7               g199(.a(new_n294), .b(new_n262), .c(new_n264), .out0(new_n295));
  oao003aa1n02x5               g200(.a(\a[30] ), .b(\b[29] ), .c(new_n288), .carry(new_n296));
  xnrc02aa1n02x5               g201(.a(\b[30] ), .b(\a[31] ), .out0(new_n297));
  nano22aa1n02x4               g202(.a(new_n295), .b(new_n296), .c(new_n297), .out0(new_n298));
  aoai13aa1n03x5               g203(.a(new_n294), .b(new_n273), .c(new_n181), .d(new_n261), .o1(new_n299));
  tech160nm_fiaoi012aa1n02p5x5 g204(.a(new_n297), .b(new_n299), .c(new_n296), .o1(new_n300));
  norp02aa1n03x5               g205(.a(new_n300), .b(new_n298), .o1(\s[31] ));
  xnrb03aa1n02x5               g206(.a(new_n112), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g207(.a(\a[3] ), .b(\b[2] ), .c(new_n112), .o1(new_n303));
  xorb03aa1n02x5               g208(.a(new_n303), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g209(.a(new_n119), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  oaoi03aa1n02x5               g210(.a(new_n102), .b(new_n103), .c(new_n119), .o1(new_n306));
  xnrb03aa1n02x5               g211(.a(new_n306), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  oaoi03aa1n02x5               g212(.a(\a[6] ), .b(\b[5] ), .c(new_n306), .o1(new_n308));
  xorb03aa1n02x5               g213(.a(new_n308), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  aoi012aa1n02x5               g214(.a(new_n99), .b(new_n308), .c(new_n100), .o1(new_n310));
  xnrb03aa1n02x5               g215(.a(new_n310), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xnrb03aa1n02x5               g216(.a(new_n123), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


