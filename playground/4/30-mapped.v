// Benchmark "adder" written by ABC on Wed Jul 17 14:14:01 2024

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
    new_n172, new_n173, new_n174, new_n176, new_n177, new_n178, new_n179,
    new_n180, new_n182, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n193, new_n194, new_n195,
    new_n196, new_n198, new_n199, new_n200, new_n201, new_n202, new_n203,
    new_n204, new_n205, new_n206, new_n207, new_n210, new_n211, new_n212,
    new_n213, new_n215, new_n216, new_n217, new_n218, new_n219, new_n221,
    new_n222, new_n223, new_n224, new_n225, new_n226, new_n228, new_n229,
    new_n230, new_n231, new_n232, new_n233, new_n234, new_n235, new_n237,
    new_n238, new_n239, new_n240, new_n241, new_n242, new_n243, new_n244,
    new_n245, new_n247, new_n248, new_n249, new_n250, new_n251, new_n252,
    new_n253, new_n254, new_n255, new_n256, new_n257, new_n258, new_n260,
    new_n261, new_n262, new_n263, new_n264, new_n265, new_n266, new_n267,
    new_n269, new_n270, new_n271, new_n272, new_n273, new_n274, new_n275,
    new_n276, new_n278, new_n279, new_n280, new_n281, new_n282, new_n283,
    new_n284, new_n285, new_n286, new_n288, new_n289, new_n290, new_n291,
    new_n292, new_n293, new_n294, new_n297, new_n298, new_n299, new_n300,
    new_n301, new_n302, new_n303, new_n305, new_n306, new_n307, new_n308,
    new_n309, new_n310, new_n311, new_n314, new_n315, new_n316, new_n317,
    new_n320, new_n322, new_n323, new_n324, new_n326;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor002aa1n20x5               g001(.a(\b[9] ), .b(\a[10] ), .o1(new_n97));
  inv000aa1d42x5               g002(.a(new_n97), .o1(new_n98));
  nand02aa1d28x5               g003(.a(\b[9] ), .b(\a[10] ), .o1(new_n99));
  inv000aa1d42x5               g004(.a(\a[9] ), .o1(new_n100));
  inv000aa1d42x5               g005(.a(\b[8] ), .o1(new_n101));
  orn002aa1n03x5               g006(.a(\a[2] ), .b(\b[1] ), .o(new_n102));
  nanp02aa1n06x5               g007(.a(\b[0] ), .b(\a[1] ), .o1(new_n103));
  aob012aa1n12x5               g008(.a(new_n103), .b(\b[1] ), .c(\a[2] ), .out0(new_n104));
  inv040aa1d32x5               g009(.a(\a[3] ), .o1(new_n105));
  inv040aa1n09x5               g010(.a(\b[2] ), .o1(new_n106));
  nand02aa1d04x5               g011(.a(new_n106), .b(new_n105), .o1(new_n107));
  nanp02aa1n02x5               g012(.a(\b[2] ), .b(\a[3] ), .o1(new_n108));
  nand22aa1n03x5               g013(.a(new_n107), .b(new_n108), .o1(new_n109));
  inv000aa1d42x5               g014(.a(\a[4] ), .o1(new_n110));
  aboi22aa1n03x5               g015(.a(\b[3] ), .b(new_n110), .c(new_n105), .d(new_n106), .out0(new_n111));
  aoai13aa1n06x5               g016(.a(new_n111), .b(new_n109), .c(new_n104), .d(new_n102), .o1(new_n112));
  tech160nm_fixnrc02aa1n02p5x5 g017(.a(\b[4] ), .b(\a[5] ), .out0(new_n113));
  nor022aa1n08x5               g018(.a(\b[6] ), .b(\a[7] ), .o1(new_n114));
  nand42aa1n04x5               g019(.a(\b[6] ), .b(\a[7] ), .o1(new_n115));
  nand42aa1n04x5               g020(.a(\b[5] ), .b(\a[6] ), .o1(new_n116));
  nanb03aa1n12x5               g021(.a(new_n114), .b(new_n116), .c(new_n115), .out0(new_n117));
  nor042aa1n02x5               g022(.a(new_n117), .b(new_n113), .o1(new_n118));
  tech160nm_finand02aa1n03p5x5 g023(.a(\b[3] ), .b(\a[4] ), .o1(new_n119));
  nor022aa1n06x5               g024(.a(\b[7] ), .b(\a[8] ), .o1(new_n120));
  nand02aa1n08x5               g025(.a(\b[7] ), .b(\a[8] ), .o1(new_n121));
  nor042aa1n04x5               g026(.a(\b[5] ), .b(\a[6] ), .o1(new_n122));
  nano23aa1n03x7               g027(.a(new_n122), .b(new_n120), .c(new_n119), .d(new_n121), .out0(new_n123));
  nanp03aa1n02x5               g028(.a(new_n112), .b(new_n118), .c(new_n123), .o1(new_n124));
  nanb02aa1n06x5               g029(.a(new_n120), .b(new_n121), .out0(new_n125));
  oab012aa1n03x5               g030(.a(new_n122), .b(\a[5] ), .c(\b[4] ), .out0(new_n126));
  norp03aa1n02x5               g031(.a(new_n117), .b(new_n125), .c(new_n126), .o1(new_n127));
  aoi012aa1n02x5               g032(.a(new_n120), .b(new_n114), .c(new_n121), .o1(new_n128));
  norb02aa1n02x5               g033(.a(new_n128), .b(new_n127), .out0(new_n129));
  nanp02aa1n03x5               g034(.a(new_n124), .b(new_n129), .o1(new_n130));
  tech160nm_fioaoi03aa1n03p5x5 g035(.a(new_n100), .b(new_n101), .c(new_n130), .o1(new_n131));
  xnbna2aa1n03x5               g036(.a(new_n131), .b(new_n98), .c(new_n99), .out0(\s[10] ));
  inv020aa1n03x5               g037(.a(new_n99), .o1(new_n133));
  nona22aa1n06x5               g038(.a(new_n131), .b(new_n133), .c(new_n97), .out0(new_n134));
  nand02aa1n12x5               g039(.a(\b[10] ), .b(\a[11] ), .o1(new_n135));
  nor002aa1d32x5               g040(.a(\b[10] ), .b(\a[11] ), .o1(new_n136));
  norb02aa1n02x5               g041(.a(new_n135), .b(new_n136), .out0(new_n137));
  xobna2aa1n03x5               g042(.a(new_n137), .b(new_n134), .c(new_n99), .out0(\s[11] ));
  nanb03aa1d24x5               g043(.a(new_n136), .b(new_n99), .c(new_n135), .out0(new_n139));
  inv000aa1d42x5               g044(.a(new_n139), .o1(new_n140));
  nor042aa1n06x5               g045(.a(\b[11] ), .b(\a[12] ), .o1(new_n141));
  nand22aa1n09x5               g046(.a(\b[11] ), .b(\a[12] ), .o1(new_n142));
  nanb02aa1d24x5               g047(.a(new_n141), .b(new_n142), .out0(new_n143));
  inv000aa1d42x5               g048(.a(new_n143), .o1(new_n144));
  aoai13aa1n02x5               g049(.a(new_n144), .b(new_n136), .c(new_n134), .d(new_n140), .o1(new_n145));
  aoi112aa1n02x5               g050(.a(new_n136), .b(new_n144), .c(new_n134), .d(new_n140), .o1(new_n146));
  norb02aa1n02x5               g051(.a(new_n145), .b(new_n146), .out0(\s[12] ));
  oai013aa1n03x4               g052(.a(new_n128), .b(new_n117), .c(new_n125), .d(new_n126), .o1(new_n148));
  aoi013aa1n09x5               g053(.a(new_n148), .b(new_n112), .c(new_n118), .d(new_n123), .o1(new_n149));
  tech160nm_fixorc02aa1n02p5x5 g054(.a(\a[9] ), .b(\b[8] ), .out0(new_n150));
  nano23aa1n02x4               g055(.a(new_n143), .b(new_n139), .c(new_n150), .d(new_n98), .out0(new_n151));
  aoi112aa1n06x5               g056(.a(new_n133), .b(new_n97), .c(new_n100), .d(new_n101), .o1(new_n152));
  aoi012aa1n02x7               g057(.a(new_n141), .b(new_n136), .c(new_n142), .o1(new_n153));
  oai013aa1d12x5               g058(.a(new_n153), .b(new_n152), .c(new_n139), .d(new_n143), .o1(new_n154));
  inv000aa1d42x5               g059(.a(new_n154), .o1(new_n155));
  oaib12aa1n02x5               g060(.a(new_n155), .b(new_n149), .c(new_n151), .out0(new_n156));
  xorb03aa1n02x5               g061(.a(new_n156), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  inv000aa1d42x5               g062(.a(\a[14] ), .o1(new_n158));
  inv000aa1d42x5               g063(.a(\a[13] ), .o1(new_n159));
  inv000aa1d42x5               g064(.a(\b[12] ), .o1(new_n160));
  oaoi03aa1n02x5               g065(.a(new_n159), .b(new_n160), .c(new_n156), .o1(new_n161));
  xorb03aa1n02x5               g066(.a(new_n161), .b(\b[13] ), .c(new_n158), .out0(\s[14] ));
  and002aa1n02x5               g067(.a(\b[13] ), .b(\a[14] ), .o(new_n163));
  nor002aa1d32x5               g068(.a(\b[14] ), .b(\a[15] ), .o1(new_n164));
  nand02aa1d08x5               g069(.a(\b[14] ), .b(\a[15] ), .o1(new_n165));
  norb02aa1d21x5               g070(.a(new_n165), .b(new_n164), .out0(new_n166));
  inv000aa1d42x5               g071(.a(new_n166), .o1(new_n167));
  nanp02aa1n03x5               g072(.a(new_n130), .b(new_n151), .o1(new_n168));
  xnrc02aa1n12x5               g073(.a(\b[12] ), .b(\a[13] ), .out0(new_n169));
  aboi22aa1d24x5               g074(.a(\b[13] ), .b(new_n158), .c(new_n159), .d(new_n160), .out0(new_n170));
  aoai13aa1n06x5               g075(.a(new_n170), .b(new_n169), .c(new_n168), .d(new_n155), .o1(new_n171));
  nona22aa1n03x5               g076(.a(new_n171), .b(new_n167), .c(new_n163), .out0(new_n172));
  inv000aa1d42x5               g077(.a(new_n164), .o1(new_n173));
  aboi22aa1n03x5               g078(.a(new_n163), .b(new_n171), .c(new_n173), .d(new_n165), .out0(new_n174));
  norb02aa1n02x5               g079(.a(new_n172), .b(new_n174), .out0(\s[15] ));
  nor002aa1n16x5               g080(.a(\b[15] ), .b(\a[16] ), .o1(new_n176));
  nand22aa1n04x5               g081(.a(\b[15] ), .b(\a[16] ), .o1(new_n177));
  norb02aa1n03x5               g082(.a(new_n177), .b(new_n176), .out0(new_n178));
  aobi12aa1n06x5               g083(.a(new_n178), .b(new_n172), .c(new_n173), .out0(new_n179));
  nona22aa1n02x5               g084(.a(new_n172), .b(new_n178), .c(new_n164), .out0(new_n180));
  norb02aa1n03x4               g085(.a(new_n180), .b(new_n179), .out0(\s[16] ));
  nor043aa1n03x5               g086(.a(new_n139), .b(new_n143), .c(new_n97), .o1(new_n182));
  tech160nm_fixnrc02aa1n02p5x5 g087(.a(\b[13] ), .b(\a[14] ), .out0(new_n183));
  nona23aa1n09x5               g088(.a(new_n177), .b(new_n165), .c(new_n164), .d(new_n176), .out0(new_n184));
  nor043aa1n06x5               g089(.a(new_n184), .b(new_n183), .c(new_n169), .o1(new_n185));
  nand23aa1n04x5               g090(.a(new_n185), .b(new_n182), .c(new_n150), .o1(new_n186));
  aoi112aa1n02x5               g091(.a(\b[14] ), .b(\a[15] ), .c(\a[16] ), .d(\b[15] ), .o1(new_n187));
  nona23aa1n02x4               g092(.a(new_n166), .b(new_n178), .c(new_n170), .d(new_n163), .out0(new_n188));
  nona22aa1n02x4               g093(.a(new_n188), .b(new_n187), .c(new_n176), .out0(new_n189));
  aoi012aa1d18x5               g094(.a(new_n189), .b(new_n154), .c(new_n185), .o1(new_n190));
  oai012aa1d24x5               g095(.a(new_n190), .b(new_n149), .c(new_n186), .o1(new_n191));
  xorb03aa1n02x5               g096(.a(new_n191), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv000aa1d42x5               g097(.a(\a[18] ), .o1(new_n193));
  inv000aa1d42x5               g098(.a(\a[17] ), .o1(new_n194));
  inv000aa1d42x5               g099(.a(\b[16] ), .o1(new_n195));
  oaoi03aa1n02x5               g100(.a(new_n194), .b(new_n195), .c(new_n191), .o1(new_n196));
  xorb03aa1n02x5               g101(.a(new_n196), .b(\b[17] ), .c(new_n193), .out0(\s[18] ));
  nor042aa1n12x5               g102(.a(\b[18] ), .b(\a[19] ), .o1(new_n198));
  nanp02aa1n04x5               g103(.a(\b[18] ), .b(\a[19] ), .o1(new_n199));
  nanb02aa1n02x5               g104(.a(new_n198), .b(new_n199), .out0(new_n200));
  inv000aa1d42x5               g105(.a(new_n200), .o1(new_n201));
  oai022aa1d24x5               g106(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n202));
  oaib12aa1n18x5               g107(.a(new_n202), .b(new_n193), .c(\b[17] ), .out0(new_n203));
  inv020aa1n04x5               g108(.a(new_n203), .o1(new_n204));
  xroi22aa1d06x4               g109(.a(new_n194), .b(\b[16] ), .c(new_n193), .d(\b[17] ), .out0(new_n205));
  aoai13aa1n06x5               g110(.a(new_n201), .b(new_n204), .c(new_n191), .d(new_n205), .o1(new_n206));
  aoi112aa1n02x5               g111(.a(new_n204), .b(new_n201), .c(new_n191), .d(new_n205), .o1(new_n207));
  norb02aa1n02x5               g112(.a(new_n206), .b(new_n207), .out0(\s[19] ));
  xnrc02aa1n02x5               g113(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  inv000aa1d42x5               g114(.a(new_n198), .o1(new_n210));
  nor042aa1n04x5               g115(.a(\b[19] ), .b(\a[20] ), .o1(new_n211));
  nand02aa1n06x5               g116(.a(\b[19] ), .b(\a[20] ), .o1(new_n212));
  nanb02aa1n02x5               g117(.a(new_n211), .b(new_n212), .out0(new_n213));
  xobna2aa1n03x5               g118(.a(new_n213), .b(new_n206), .c(new_n210), .out0(\s[20] ));
  nona23aa1n09x5               g119(.a(new_n212), .b(new_n199), .c(new_n198), .d(new_n211), .out0(new_n215));
  norb02aa1n03x5               g120(.a(new_n205), .b(new_n215), .out0(new_n216));
  aoi012aa1n12x5               g121(.a(new_n211), .b(new_n198), .c(new_n212), .o1(new_n217));
  oai012aa1n12x5               g122(.a(new_n217), .b(new_n215), .c(new_n203), .o1(new_n218));
  tech160nm_fiaoi012aa1n04x5   g123(.a(new_n218), .b(new_n191), .c(new_n216), .o1(new_n219));
  xnrb03aa1n02x5               g124(.a(new_n219), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor042aa1n09x5               g125(.a(\b[20] ), .b(\a[21] ), .o1(new_n221));
  inv000aa1d42x5               g126(.a(new_n221), .o1(new_n222));
  xnrc02aa1n12x5               g127(.a(\b[20] ), .b(\a[21] ), .out0(new_n223));
  xnrc02aa1n12x5               g128(.a(\b[21] ), .b(\a[22] ), .out0(new_n224));
  oaoi13aa1n02x7               g129(.a(new_n224), .b(new_n222), .c(new_n219), .d(new_n223), .o1(new_n225));
  oai112aa1n02x7               g130(.a(new_n224), .b(new_n222), .c(new_n219), .d(new_n223), .o1(new_n226));
  norb02aa1n03x4               g131(.a(new_n226), .b(new_n225), .out0(\s[22] ));
  inv000aa1n02x5               g132(.a(new_n216), .o1(new_n228));
  nona32aa1n06x5               g133(.a(new_n191), .b(new_n224), .c(new_n223), .d(new_n228), .out0(new_n229));
  nor042aa1n06x5               g134(.a(new_n224), .b(new_n223), .o1(new_n230));
  oao003aa1n12x5               g135(.a(\a[22] ), .b(\b[21] ), .c(new_n222), .carry(new_n231));
  inv000aa1d42x5               g136(.a(new_n231), .o1(new_n232));
  aoi012aa1d18x5               g137(.a(new_n232), .b(new_n218), .c(new_n230), .o1(new_n233));
  xnrc02aa1n12x5               g138(.a(\b[22] ), .b(\a[23] ), .out0(new_n234));
  inv000aa1d42x5               g139(.a(new_n234), .o1(new_n235));
  xnbna2aa1n03x5               g140(.a(new_n235), .b(new_n229), .c(new_n233), .out0(\s[23] ));
  nor042aa1n03x5               g141(.a(\b[22] ), .b(\a[23] ), .o1(new_n237));
  inv000aa1d42x5               g142(.a(new_n237), .o1(new_n238));
  oaoi13aa1n09x5               g143(.a(new_n228), .b(new_n190), .c(new_n149), .d(new_n186), .o1(new_n239));
  inv000aa1d42x5               g144(.a(new_n233), .o1(new_n240));
  aoai13aa1n03x5               g145(.a(new_n235), .b(new_n240), .c(new_n239), .d(new_n230), .o1(new_n241));
  tech160nm_fixnrc02aa1n04x5   g146(.a(\b[23] ), .b(\a[24] ), .out0(new_n242));
  tech160nm_fiaoi012aa1n02p5x5 g147(.a(new_n242), .b(new_n241), .c(new_n238), .o1(new_n243));
  tech160nm_fiaoi012aa1n05x5   g148(.a(new_n234), .b(new_n229), .c(new_n233), .o1(new_n244));
  nano22aa1n03x5               g149(.a(new_n244), .b(new_n238), .c(new_n242), .out0(new_n245));
  norp02aa1n03x5               g150(.a(new_n243), .b(new_n245), .o1(\s[24] ));
  nor042aa1n02x5               g151(.a(new_n242), .b(new_n234), .o1(new_n247));
  nand02aa1d04x5               g152(.a(new_n247), .b(new_n230), .o1(new_n248));
  nona22aa1n06x5               g153(.a(new_n191), .b(new_n228), .c(new_n248), .out0(new_n249));
  nano23aa1n09x5               g154(.a(new_n198), .b(new_n211), .c(new_n212), .d(new_n199), .out0(new_n250));
  inv020aa1n04x5               g155(.a(new_n217), .o1(new_n251));
  aoai13aa1n06x5               g156(.a(new_n230), .b(new_n251), .c(new_n250), .d(new_n204), .o1(new_n252));
  inv030aa1n03x5               g157(.a(new_n247), .o1(new_n253));
  oao003aa1n03x5               g158(.a(\a[24] ), .b(\b[23] ), .c(new_n238), .carry(new_n254));
  aoai13aa1n12x5               g159(.a(new_n254), .b(new_n253), .c(new_n252), .d(new_n231), .o1(new_n255));
  inv000aa1d42x5               g160(.a(new_n255), .o1(new_n256));
  xnrc02aa1n12x5               g161(.a(\b[24] ), .b(\a[25] ), .out0(new_n257));
  inv000aa1d42x5               g162(.a(new_n257), .o1(new_n258));
  xnbna2aa1n03x5               g163(.a(new_n258), .b(new_n249), .c(new_n256), .out0(\s[25] ));
  nor042aa1n03x5               g164(.a(\b[24] ), .b(\a[25] ), .o1(new_n260));
  inv000aa1d42x5               g165(.a(new_n260), .o1(new_n261));
  inv000aa1d42x5               g166(.a(new_n248), .o1(new_n262));
  aoai13aa1n03x5               g167(.a(new_n258), .b(new_n255), .c(new_n239), .d(new_n262), .o1(new_n263));
  xnrc02aa1n02x5               g168(.a(\b[25] ), .b(\a[26] ), .out0(new_n264));
  tech160nm_fiaoi012aa1n02p5x5 g169(.a(new_n264), .b(new_n263), .c(new_n261), .o1(new_n265));
  tech160nm_fiaoi012aa1n05x5   g170(.a(new_n257), .b(new_n249), .c(new_n256), .o1(new_n266));
  nano22aa1n03x5               g171(.a(new_n266), .b(new_n261), .c(new_n264), .out0(new_n267));
  norp02aa1n03x5               g172(.a(new_n265), .b(new_n267), .o1(\s[26] ));
  nor022aa1n04x5               g173(.a(new_n264), .b(new_n257), .o1(new_n269));
  inv000aa1n02x5               g174(.a(new_n269), .o1(new_n270));
  nona32aa1n09x5               g175(.a(new_n191), .b(new_n270), .c(new_n248), .d(new_n228), .out0(new_n271));
  oao003aa1n02x5               g176(.a(\a[26] ), .b(\b[25] ), .c(new_n261), .carry(new_n272));
  aobi12aa1n12x5               g177(.a(new_n272), .b(new_n255), .c(new_n269), .out0(new_n273));
  nor042aa1n03x5               g178(.a(\b[26] ), .b(\a[27] ), .o1(new_n274));
  nanp02aa1n02x5               g179(.a(\b[26] ), .b(\a[27] ), .o1(new_n275));
  norb02aa1n02x5               g180(.a(new_n275), .b(new_n274), .out0(new_n276));
  xnbna2aa1n03x5               g181(.a(new_n276), .b(new_n273), .c(new_n271), .out0(\s[27] ));
  inv000aa1d42x5               g182(.a(new_n274), .o1(new_n278));
  nano22aa1n02x4               g183(.a(new_n270), .b(new_n230), .c(new_n247), .out0(new_n279));
  aoai13aa1n06x5               g184(.a(new_n247), .b(new_n232), .c(new_n218), .d(new_n230), .o1(new_n280));
  aoai13aa1n06x5               g185(.a(new_n272), .b(new_n270), .c(new_n280), .d(new_n254), .o1(new_n281));
  aoai13aa1n03x5               g186(.a(new_n275), .b(new_n281), .c(new_n239), .d(new_n279), .o1(new_n282));
  xnrc02aa1n02x5               g187(.a(\b[27] ), .b(\a[28] ), .out0(new_n283));
  tech160nm_fiaoi012aa1n02p5x5 g188(.a(new_n283), .b(new_n282), .c(new_n278), .o1(new_n284));
  aoi022aa1n02x7               g189(.a(new_n273), .b(new_n271), .c(\a[27] ), .d(\b[26] ), .o1(new_n285));
  nano22aa1n03x5               g190(.a(new_n285), .b(new_n278), .c(new_n283), .out0(new_n286));
  norp02aa1n03x5               g191(.a(new_n284), .b(new_n286), .o1(\s[28] ));
  nano22aa1n02x4               g192(.a(new_n283), .b(new_n278), .c(new_n275), .out0(new_n288));
  aoai13aa1n03x5               g193(.a(new_n288), .b(new_n281), .c(new_n239), .d(new_n279), .o1(new_n289));
  oao003aa1n02x5               g194(.a(\a[28] ), .b(\b[27] ), .c(new_n278), .carry(new_n290));
  xnrc02aa1n02x5               g195(.a(\b[28] ), .b(\a[29] ), .out0(new_n291));
  tech160nm_fiaoi012aa1n02p5x5 g196(.a(new_n291), .b(new_n289), .c(new_n290), .o1(new_n292));
  aobi12aa1n02x7               g197(.a(new_n288), .b(new_n273), .c(new_n271), .out0(new_n293));
  nano22aa1n03x5               g198(.a(new_n293), .b(new_n290), .c(new_n291), .out0(new_n294));
  norp02aa1n03x5               g199(.a(new_n292), .b(new_n294), .o1(\s[29] ));
  xorb03aa1n02x5               g200(.a(new_n103), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g201(.a(new_n276), .b(new_n291), .c(new_n283), .out0(new_n297));
  aoai13aa1n03x5               g202(.a(new_n297), .b(new_n281), .c(new_n239), .d(new_n279), .o1(new_n298));
  oao003aa1n02x5               g203(.a(\a[29] ), .b(\b[28] ), .c(new_n290), .carry(new_n299));
  xnrc02aa1n02x5               g204(.a(\b[29] ), .b(\a[30] ), .out0(new_n300));
  tech160nm_fiaoi012aa1n02p5x5 g205(.a(new_n300), .b(new_n298), .c(new_n299), .o1(new_n301));
  aobi12aa1n02x7               g206(.a(new_n297), .b(new_n273), .c(new_n271), .out0(new_n302));
  nano22aa1n02x4               g207(.a(new_n302), .b(new_n299), .c(new_n300), .out0(new_n303));
  norp02aa1n03x5               g208(.a(new_n301), .b(new_n303), .o1(\s[30] ));
  xnrc02aa1n02x5               g209(.a(\b[30] ), .b(\a[31] ), .out0(new_n305));
  norb03aa1n02x5               g210(.a(new_n288), .b(new_n300), .c(new_n291), .out0(new_n306));
  aoai13aa1n03x5               g211(.a(new_n306), .b(new_n281), .c(new_n239), .d(new_n279), .o1(new_n307));
  oao003aa1n02x5               g212(.a(\a[30] ), .b(\b[29] ), .c(new_n299), .carry(new_n308));
  tech160nm_fiaoi012aa1n02p5x5 g213(.a(new_n305), .b(new_n307), .c(new_n308), .o1(new_n309));
  aobi12aa1n02x7               g214(.a(new_n306), .b(new_n273), .c(new_n271), .out0(new_n310));
  nano22aa1n03x5               g215(.a(new_n310), .b(new_n305), .c(new_n308), .out0(new_n311));
  norp02aa1n03x5               g216(.a(new_n309), .b(new_n311), .o1(\s[31] ));
  xobna2aa1n03x5               g217(.a(new_n109), .b(new_n104), .c(new_n102), .out0(\s[3] ));
  xnrc02aa1n02x5               g218(.a(\b[3] ), .b(\a[4] ), .out0(new_n314));
  aoai13aa1n02x5               g219(.a(new_n107), .b(new_n109), .c(new_n104), .d(new_n102), .o1(new_n315));
  nanp02aa1n02x5               g220(.a(new_n112), .b(new_n119), .o1(new_n316));
  aoib12aa1n02x5               g221(.a(new_n316), .b(new_n110), .c(\b[3] ), .out0(new_n317));
  aoib12aa1n02x5               g222(.a(new_n317), .b(new_n314), .c(new_n315), .out0(\s[4] ));
  xnbna2aa1n03x5               g223(.a(new_n113), .b(new_n112), .c(new_n119), .out0(\s[5] ));
  oaoi03aa1n02x5               g224(.a(\a[5] ), .b(\b[4] ), .c(new_n316), .o1(new_n320));
  xorb03aa1n02x5               g225(.a(new_n320), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  norb02aa1n02x5               g226(.a(new_n115), .b(new_n114), .out0(new_n322));
  oaoi13aa1n02x5               g227(.a(new_n322), .b(new_n116), .c(new_n320), .d(new_n122), .o1(new_n323));
  oai112aa1n02x5               g228(.a(new_n116), .b(new_n322), .c(new_n320), .d(new_n122), .o1(new_n324));
  norb02aa1n02x5               g229(.a(new_n324), .b(new_n323), .out0(\s[7] ));
  orn002aa1n02x5               g230(.a(\a[7] ), .b(\b[6] ), .o(new_n326));
  xobna2aa1n03x5               g231(.a(new_n125), .b(new_n324), .c(new_n326), .out0(\s[8] ));
  xnbna2aa1n03x5               g232(.a(new_n150), .b(new_n124), .c(new_n129), .out0(\s[9] ));
endmodule


