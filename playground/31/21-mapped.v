// Benchmark "adder" written by ABC on Thu Jul 18 04:01:07 2024

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
    new_n134, new_n135, new_n136, new_n137, new_n138, new_n139, new_n140,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n152, new_n153, new_n154, new_n156, new_n157,
    new_n158, new_n159, new_n160, new_n162, new_n163, new_n164, new_n165,
    new_n166, new_n168, new_n169, new_n170, new_n171, new_n172, new_n173,
    new_n174, new_n175, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n183, new_n184, new_n185, new_n186, new_n188, new_n189,
    new_n190, new_n191, new_n192, new_n193, new_n194, new_n195, new_n198,
    new_n199, new_n200, new_n201, new_n202, new_n203, new_n205, new_n206,
    new_n207, new_n208, new_n209, new_n210, new_n211, new_n212, new_n213,
    new_n215, new_n216, new_n217, new_n218, new_n219, new_n221, new_n222,
    new_n223, new_n224, new_n225, new_n226, new_n227, new_n228, new_n230,
    new_n231, new_n232, new_n233, new_n234, new_n236, new_n237, new_n238,
    new_n239, new_n240, new_n241, new_n242, new_n243, new_n244, new_n245,
    new_n246, new_n247, new_n249, new_n250, new_n251, new_n252, new_n253,
    new_n255, new_n256, new_n257, new_n258, new_n259, new_n260, new_n261,
    new_n262, new_n263, new_n264, new_n265, new_n266, new_n267, new_n268,
    new_n269, new_n270, new_n271, new_n272, new_n274, new_n275, new_n276,
    new_n277, new_n278, new_n279, new_n280, new_n281, new_n282, new_n283,
    new_n284, new_n285, new_n287, new_n288, new_n289, new_n290, new_n291,
    new_n292, new_n293, new_n296, new_n297, new_n298, new_n299, new_n300,
    new_n301, new_n302, new_n304, new_n305, new_n306, new_n307, new_n308,
    new_n309, new_n310, new_n313, new_n316, new_n317, new_n318, new_n321;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  orn002aa1n02x5               g001(.a(\a[9] ), .b(\b[8] ), .o(new_n97));
  aoi112aa1n09x5               g002(.a(\b[6] ), .b(\a[7] ), .c(\a[8] ), .d(\b[7] ), .o1(new_n98));
  norp02aa1n02x5               g003(.a(\b[7] ), .b(\a[8] ), .o1(new_n99));
  nor042aa1n12x5               g004(.a(\b[5] ), .b(\a[6] ), .o1(new_n100));
  oab012aa1n03x5               g005(.a(new_n100), .b(\a[5] ), .c(\b[4] ), .out0(new_n101));
  tech160nm_fixorc02aa1n05x5   g006(.a(\a[8] ), .b(\b[7] ), .out0(new_n102));
  nor022aa1n16x5               g007(.a(\b[6] ), .b(\a[7] ), .o1(new_n103));
  nand42aa1n06x5               g008(.a(\b[6] ), .b(\a[7] ), .o1(new_n104));
  norb02aa1n06x4               g009(.a(new_n104), .b(new_n103), .out0(new_n105));
  and002aa1n12x5               g010(.a(\b[5] ), .b(\a[6] ), .o(new_n106));
  nona23aa1n02x5               g011(.a(new_n102), .b(new_n105), .c(new_n101), .d(new_n106), .out0(new_n107));
  nona22aa1n09x5               g012(.a(new_n107), .b(new_n99), .c(new_n98), .out0(new_n108));
  nor022aa1n16x5               g013(.a(\b[3] ), .b(\a[4] ), .o1(new_n109));
  nand22aa1n12x5               g014(.a(\b[3] ), .b(\a[4] ), .o1(new_n110));
  norp02aa1n24x5               g015(.a(\b[2] ), .b(\a[3] ), .o1(new_n111));
  nand42aa1n03x5               g016(.a(\b[2] ), .b(\a[3] ), .o1(new_n112));
  nona23aa1n09x5               g017(.a(new_n112), .b(new_n110), .c(new_n109), .d(new_n111), .out0(new_n113));
  nanp02aa1n02x5               g018(.a(\b[1] ), .b(\a[2] ), .o1(new_n114));
  nand02aa1d04x5               g019(.a(\b[0] ), .b(\a[1] ), .o1(new_n115));
  nor022aa1n04x5               g020(.a(\b[1] ), .b(\a[2] ), .o1(new_n116));
  oai012aa1n12x5               g021(.a(new_n114), .b(new_n116), .c(new_n115), .o1(new_n117));
  tech160nm_fiao0012aa1n02p5x5 g022(.a(new_n109), .b(new_n111), .c(new_n110), .o(new_n118));
  oabi12aa1n18x5               g023(.a(new_n118), .b(new_n113), .c(new_n117), .out0(new_n119));
  xnrc02aa1n02x5               g024(.a(\b[7] ), .b(\a[8] ), .out0(new_n120));
  nor042aa1n06x5               g025(.a(new_n106), .b(new_n100), .o1(new_n121));
  xorc02aa1n12x5               g026(.a(\a[5] ), .b(\b[4] ), .out0(new_n122));
  nano32aa1n03x7               g027(.a(new_n120), .b(new_n122), .c(new_n105), .d(new_n121), .out0(new_n123));
  tech160nm_fixorc02aa1n03p5x5 g028(.a(\a[9] ), .b(\b[8] ), .out0(new_n124));
  aoai13aa1n06x5               g029(.a(new_n124), .b(new_n108), .c(new_n123), .d(new_n119), .o1(new_n125));
  tech160nm_fixorc02aa1n03p5x5 g030(.a(\a[10] ), .b(\b[9] ), .out0(new_n126));
  xnbna2aa1n03x5               g031(.a(new_n126), .b(new_n125), .c(new_n97), .out0(\s[10] ));
  inv000aa1d42x5               g032(.a(\b[9] ), .o1(new_n128));
  nanb02aa1n02x5               g033(.a(\a[10] ), .b(new_n128), .out0(new_n129));
  and002aa1n02x5               g034(.a(\b[9] ), .b(\a[10] ), .o(new_n130));
  aoi013aa1n06x4               g035(.a(new_n130), .b(new_n125), .c(new_n129), .d(new_n97), .o1(new_n131));
  xorb03aa1n02x5               g036(.a(new_n131), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  nor002aa1d32x5               g037(.a(\b[10] ), .b(\a[11] ), .o1(new_n133));
  nand42aa1d28x5               g038(.a(\b[10] ), .b(\a[11] ), .o1(new_n134));
  norb02aa1n02x5               g039(.a(new_n134), .b(new_n133), .out0(new_n135));
  nor002aa1d32x5               g040(.a(\b[11] ), .b(\a[12] ), .o1(new_n136));
  nand42aa1n20x5               g041(.a(\b[11] ), .b(\a[12] ), .o1(new_n137));
  norb02aa1n02x5               g042(.a(new_n137), .b(new_n136), .out0(new_n138));
  aoi112aa1n03x5               g043(.a(new_n138), .b(new_n133), .c(new_n131), .d(new_n135), .o1(new_n139));
  aoai13aa1n02x7               g044(.a(new_n138), .b(new_n133), .c(new_n131), .d(new_n135), .o1(new_n140));
  norb02aa1n02x7               g045(.a(new_n140), .b(new_n139), .out0(\s[12] ));
  nona23aa1d18x5               g046(.a(new_n137), .b(new_n134), .c(new_n133), .d(new_n136), .out0(new_n142));
  nano22aa1n02x4               g047(.a(new_n142), .b(new_n124), .c(new_n126), .out0(new_n143));
  aoai13aa1n06x5               g048(.a(new_n143), .b(new_n108), .c(new_n123), .d(new_n119), .o1(new_n144));
  tech160nm_fioai012aa1n03p5x5 g049(.a(new_n137), .b(new_n136), .c(new_n133), .o1(new_n145));
  oai022aa1d18x5               g050(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n146));
  oaib12aa1n18x5               g051(.a(new_n146), .b(new_n128), .c(\a[10] ), .out0(new_n147));
  oai012aa1d24x5               g052(.a(new_n145), .b(new_n142), .c(new_n147), .o1(new_n148));
  inv000aa1d42x5               g053(.a(new_n148), .o1(new_n149));
  nanp02aa1n02x5               g054(.a(new_n144), .b(new_n149), .o1(new_n150));
  xorb03aa1n02x5               g055(.a(new_n150), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  nor002aa1d32x5               g056(.a(\b[12] ), .b(\a[13] ), .o1(new_n152));
  nand42aa1d28x5               g057(.a(\b[12] ), .b(\a[13] ), .o1(new_n153));
  aoi012aa1n02x5               g058(.a(new_n152), .b(new_n150), .c(new_n153), .o1(new_n154));
  xnrb03aa1n02x5               g059(.a(new_n154), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  nor002aa1d32x5               g060(.a(\b[13] ), .b(\a[14] ), .o1(new_n156));
  nand42aa1d28x5               g061(.a(\b[13] ), .b(\a[14] ), .o1(new_n157));
  nona23aa1n09x5               g062(.a(new_n157), .b(new_n153), .c(new_n152), .d(new_n156), .out0(new_n158));
  tech160nm_fioai012aa1n04x5   g063(.a(new_n157), .b(new_n156), .c(new_n152), .o1(new_n159));
  aoai13aa1n04x5               g064(.a(new_n159), .b(new_n158), .c(new_n144), .d(new_n149), .o1(new_n160));
  xorb03aa1n02x5               g065(.a(new_n160), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  norp02aa1n02x5               g066(.a(\b[14] ), .b(\a[15] ), .o1(new_n162));
  tech160nm_fixorc02aa1n03p5x5 g067(.a(\a[15] ), .b(\b[14] ), .out0(new_n163));
  tech160nm_fixorc02aa1n03p5x5 g068(.a(\a[16] ), .b(\b[15] ), .out0(new_n164));
  aoi112aa1n02x5               g069(.a(new_n164), .b(new_n162), .c(new_n160), .d(new_n163), .o1(new_n165));
  aoai13aa1n02x5               g070(.a(new_n164), .b(new_n162), .c(new_n160), .d(new_n163), .o1(new_n166));
  norb02aa1n02x7               g071(.a(new_n166), .b(new_n165), .out0(\s[16] ));
  nano23aa1n03x7               g072(.a(new_n133), .b(new_n136), .c(new_n137), .d(new_n134), .out0(new_n168));
  nano23aa1n06x5               g073(.a(new_n152), .b(new_n156), .c(new_n157), .d(new_n153), .out0(new_n169));
  nanp03aa1n03x5               g074(.a(new_n169), .b(new_n163), .c(new_n164), .o1(new_n170));
  nano32aa1n09x5               g075(.a(new_n170), .b(new_n168), .c(new_n126), .d(new_n124), .out0(new_n171));
  aoai13aa1n12x5               g076(.a(new_n171), .b(new_n108), .c(new_n123), .d(new_n119), .o1(new_n172));
  tech160nm_fixnrc02aa1n02p5x5 g077(.a(\b[14] ), .b(\a[15] ), .out0(new_n173));
  tech160nm_fixnrc02aa1n03p5x5 g078(.a(\b[15] ), .b(\a[16] ), .out0(new_n174));
  nor043aa1n03x5               g079(.a(new_n158), .b(new_n174), .c(new_n173), .o1(new_n175));
  aoi112aa1n03x5               g080(.a(\b[14] ), .b(\a[15] ), .c(\a[16] ), .d(\b[15] ), .o1(new_n176));
  nor042aa1n03x5               g081(.a(\b[15] ), .b(\a[16] ), .o1(new_n177));
  inv000aa1d42x5               g082(.a(new_n177), .o1(new_n178));
  oai013aa1n03x5               g083(.a(new_n178), .b(new_n173), .c(new_n174), .d(new_n159), .o1(new_n179));
  aoi112aa1n09x5               g084(.a(new_n179), .b(new_n176), .c(new_n148), .d(new_n175), .o1(new_n180));
  nand02aa1d10x5               g085(.a(new_n172), .b(new_n180), .o1(new_n181));
  xorb03aa1n02x5               g086(.a(new_n181), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv040aa1d32x5               g087(.a(\a[18] ), .o1(new_n183));
  inv000aa1d42x5               g088(.a(\a[17] ), .o1(new_n184));
  inv000aa1d42x5               g089(.a(\b[16] ), .o1(new_n185));
  oaoi03aa1n03x5               g090(.a(new_n184), .b(new_n185), .c(new_n181), .o1(new_n186));
  xorb03aa1n02x5               g091(.a(new_n186), .b(\b[17] ), .c(new_n183), .out0(\s[18] ));
  xroi22aa1d06x4               g092(.a(new_n184), .b(\b[16] ), .c(new_n183), .d(\b[17] ), .out0(new_n188));
  nanp02aa1n02x5               g093(.a(new_n185), .b(new_n184), .o1(new_n189));
  oaoi03aa1n12x5               g094(.a(\a[18] ), .b(\b[17] ), .c(new_n189), .o1(new_n190));
  nor002aa1n10x5               g095(.a(\b[18] ), .b(\a[19] ), .o1(new_n191));
  nand22aa1n04x5               g096(.a(\b[18] ), .b(\a[19] ), .o1(new_n192));
  norb02aa1n02x5               g097(.a(new_n192), .b(new_n191), .out0(new_n193));
  aoai13aa1n06x5               g098(.a(new_n193), .b(new_n190), .c(new_n181), .d(new_n188), .o1(new_n194));
  aoi112aa1n02x5               g099(.a(new_n193), .b(new_n190), .c(new_n181), .d(new_n188), .o1(new_n195));
  norb02aa1n02x7               g100(.a(new_n194), .b(new_n195), .out0(\s[19] ));
  xnrc02aa1n02x5               g101(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor002aa1n12x5               g102(.a(\b[19] ), .b(\a[20] ), .o1(new_n198));
  nand22aa1n06x5               g103(.a(\b[19] ), .b(\a[20] ), .o1(new_n199));
  norb02aa1n02x5               g104(.a(new_n199), .b(new_n198), .out0(new_n200));
  nona22aa1n02x5               g105(.a(new_n194), .b(new_n200), .c(new_n191), .out0(new_n201));
  inv000aa1d42x5               g106(.a(new_n200), .o1(new_n202));
  oaoi13aa1n06x5               g107(.a(new_n202), .b(new_n194), .c(\a[19] ), .d(\b[18] ), .o1(new_n203));
  norb02aa1n03x4               g108(.a(new_n201), .b(new_n203), .out0(\s[20] ));
  nano23aa1n06x5               g109(.a(new_n191), .b(new_n198), .c(new_n199), .d(new_n192), .out0(new_n205));
  nanp02aa1n02x5               g110(.a(new_n188), .b(new_n205), .o1(new_n206));
  oai022aa1n04x5               g111(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n207));
  oaib12aa1n06x5               g112(.a(new_n207), .b(new_n183), .c(\b[17] ), .out0(new_n208));
  nona23aa1n12x5               g113(.a(new_n199), .b(new_n192), .c(new_n191), .d(new_n198), .out0(new_n209));
  aoi012aa1n09x5               g114(.a(new_n198), .b(new_n191), .c(new_n199), .o1(new_n210));
  oai012aa1d24x5               g115(.a(new_n210), .b(new_n209), .c(new_n208), .o1(new_n211));
  inv000aa1d42x5               g116(.a(new_n211), .o1(new_n212));
  aoai13aa1n06x5               g117(.a(new_n212), .b(new_n206), .c(new_n172), .d(new_n180), .o1(new_n213));
  xorb03aa1n02x5               g118(.a(new_n213), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor042aa1n04x5               g119(.a(\b[20] ), .b(\a[21] ), .o1(new_n215));
  xorc02aa1n02x5               g120(.a(\a[21] ), .b(\b[20] ), .out0(new_n216));
  xorc02aa1n02x5               g121(.a(\a[22] ), .b(\b[21] ), .out0(new_n217));
  aoi112aa1n02x5               g122(.a(new_n215), .b(new_n217), .c(new_n213), .d(new_n216), .o1(new_n218));
  aoai13aa1n03x5               g123(.a(new_n217), .b(new_n215), .c(new_n213), .d(new_n216), .o1(new_n219));
  norb02aa1n02x7               g124(.a(new_n219), .b(new_n218), .out0(\s[22] ));
  inv000aa1d42x5               g125(.a(\a[21] ), .o1(new_n221));
  inv040aa1d32x5               g126(.a(\a[22] ), .o1(new_n222));
  xroi22aa1d06x4               g127(.a(new_n221), .b(\b[20] ), .c(new_n222), .d(\b[21] ), .out0(new_n223));
  inv000aa1d42x5               g128(.a(\b[21] ), .o1(new_n224));
  oao003aa1n02x5               g129(.a(new_n222), .b(new_n224), .c(new_n215), .carry(new_n225));
  aoi012aa1n02x5               g130(.a(new_n225), .b(new_n211), .c(new_n223), .o1(new_n226));
  nand43aa1n02x5               g131(.a(new_n223), .b(new_n188), .c(new_n205), .o1(new_n227));
  aoai13aa1n06x5               g132(.a(new_n226), .b(new_n227), .c(new_n172), .d(new_n180), .o1(new_n228));
  xorb03aa1n02x5               g133(.a(new_n228), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g134(.a(\b[22] ), .b(\a[23] ), .o1(new_n230));
  xorc02aa1n03x5               g135(.a(\a[23] ), .b(\b[22] ), .out0(new_n231));
  xorc02aa1n02x5               g136(.a(\a[24] ), .b(\b[23] ), .out0(new_n232));
  aoi112aa1n02x5               g137(.a(new_n230), .b(new_n232), .c(new_n228), .d(new_n231), .o1(new_n233));
  aoai13aa1n03x5               g138(.a(new_n232), .b(new_n230), .c(new_n228), .d(new_n231), .o1(new_n234));
  norb02aa1n02x7               g139(.a(new_n234), .b(new_n233), .out0(\s[24] ));
  and002aa1n02x7               g140(.a(new_n232), .b(new_n231), .o(new_n236));
  inv000aa1n02x5               g141(.a(new_n236), .o1(new_n237));
  nano32aa1n02x4               g142(.a(new_n237), .b(new_n223), .c(new_n188), .d(new_n205), .out0(new_n238));
  inv000aa1n06x5               g143(.a(new_n210), .o1(new_n239));
  aoai13aa1n06x5               g144(.a(new_n223), .b(new_n239), .c(new_n205), .d(new_n190), .o1(new_n240));
  inv000aa1n02x5               g145(.a(new_n225), .o1(new_n241));
  aoi112aa1n02x5               g146(.a(\b[22] ), .b(\a[23] ), .c(\a[24] ), .d(\b[23] ), .o1(new_n242));
  oab012aa1n02x4               g147(.a(new_n242), .b(\a[24] ), .c(\b[23] ), .out0(new_n243));
  aoai13aa1n06x5               g148(.a(new_n243), .b(new_n237), .c(new_n240), .d(new_n241), .o1(new_n244));
  xorc02aa1n02x5               g149(.a(\a[25] ), .b(\b[24] ), .out0(new_n245));
  aoai13aa1n06x5               g150(.a(new_n245), .b(new_n244), .c(new_n181), .d(new_n238), .o1(new_n246));
  aoi112aa1n02x5               g151(.a(new_n245), .b(new_n244), .c(new_n181), .d(new_n238), .o1(new_n247));
  norb02aa1n02x7               g152(.a(new_n246), .b(new_n247), .out0(\s[25] ));
  nor042aa1n03x5               g153(.a(\b[24] ), .b(\a[25] ), .o1(new_n249));
  xorc02aa1n02x5               g154(.a(\a[26] ), .b(\b[25] ), .out0(new_n250));
  nona22aa1n02x5               g155(.a(new_n246), .b(new_n250), .c(new_n249), .out0(new_n251));
  inv000aa1d42x5               g156(.a(new_n249), .o1(new_n252));
  aobi12aa1n06x5               g157(.a(new_n250), .b(new_n246), .c(new_n252), .out0(new_n253));
  norb02aa1n03x4               g158(.a(new_n251), .b(new_n253), .out0(\s[26] ));
  oai022aa1n02x5               g159(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n255));
  nanb02aa1n03x5               g160(.a(new_n103), .b(new_n104), .out0(new_n256));
  inv030aa1n04x5               g161(.a(new_n106), .o1(new_n257));
  nano23aa1n02x4               g162(.a(new_n120), .b(new_n256), .c(new_n257), .d(new_n255), .out0(new_n258));
  nor043aa1n02x5               g163(.a(new_n258), .b(new_n99), .c(new_n98), .o1(new_n259));
  oab012aa1n02x4               g164(.a(new_n118), .b(new_n113), .c(new_n117), .out0(new_n260));
  nona23aa1n02x4               g165(.a(new_n122), .b(new_n121), .c(new_n120), .d(new_n256), .out0(new_n261));
  tech160nm_fioai012aa1n04x5   g166(.a(new_n259), .b(new_n260), .c(new_n261), .o1(new_n262));
  tech160nm_finand02aa1n05x5   g167(.a(new_n148), .b(new_n175), .o1(new_n263));
  nona22aa1n02x4               g168(.a(new_n263), .b(new_n179), .c(new_n176), .out0(new_n264));
  inv000aa1d42x5               g169(.a(\a[25] ), .o1(new_n265));
  inv040aa1d32x5               g170(.a(\a[26] ), .o1(new_n266));
  xroi22aa1d06x4               g171(.a(new_n265), .b(\b[24] ), .c(new_n266), .d(\b[25] ), .out0(new_n267));
  nano22aa1n03x7               g172(.a(new_n227), .b(new_n236), .c(new_n267), .out0(new_n268));
  aoai13aa1n06x5               g173(.a(new_n268), .b(new_n264), .c(new_n262), .d(new_n171), .o1(new_n269));
  oao003aa1n02x5               g174(.a(\a[26] ), .b(\b[25] ), .c(new_n252), .carry(new_n270));
  aobi12aa1n06x5               g175(.a(new_n270), .b(new_n244), .c(new_n267), .out0(new_n271));
  xorc02aa1n12x5               g176(.a(\a[27] ), .b(\b[26] ), .out0(new_n272));
  xnbna2aa1n03x5               g177(.a(new_n272), .b(new_n271), .c(new_n269), .out0(\s[27] ));
  nor042aa1n03x5               g178(.a(\b[26] ), .b(\a[27] ), .o1(new_n274));
  inv000aa1d42x5               g179(.a(new_n274), .o1(new_n275));
  aobi12aa1n02x7               g180(.a(new_n272), .b(new_n271), .c(new_n269), .out0(new_n276));
  xnrc02aa1n02x5               g181(.a(\b[27] ), .b(\a[28] ), .out0(new_n277));
  nano22aa1n03x5               g182(.a(new_n276), .b(new_n275), .c(new_n277), .out0(new_n278));
  inv020aa1n02x5               g183(.a(new_n268), .o1(new_n279));
  aoi012aa1n06x5               g184(.a(new_n279), .b(new_n172), .c(new_n180), .o1(new_n280));
  aoai13aa1n02x7               g185(.a(new_n236), .b(new_n225), .c(new_n211), .d(new_n223), .o1(new_n281));
  inv000aa1d42x5               g186(.a(new_n267), .o1(new_n282));
  aoai13aa1n06x5               g187(.a(new_n270), .b(new_n282), .c(new_n281), .d(new_n243), .o1(new_n283));
  oaih12aa1n02x5               g188(.a(new_n272), .b(new_n283), .c(new_n280), .o1(new_n284));
  tech160nm_fiaoi012aa1n02p5x5 g189(.a(new_n277), .b(new_n284), .c(new_n275), .o1(new_n285));
  norp02aa1n03x5               g190(.a(new_n285), .b(new_n278), .o1(\s[28] ));
  norb02aa1n02x5               g191(.a(new_n272), .b(new_n277), .out0(new_n287));
  oaih12aa1n02x5               g192(.a(new_n287), .b(new_n283), .c(new_n280), .o1(new_n288));
  oao003aa1n02x5               g193(.a(\a[28] ), .b(\b[27] ), .c(new_n275), .carry(new_n289));
  xnrc02aa1n02x5               g194(.a(\b[28] ), .b(\a[29] ), .out0(new_n290));
  tech160nm_fiaoi012aa1n02p5x5 g195(.a(new_n290), .b(new_n288), .c(new_n289), .o1(new_n291));
  aobi12aa1n02x7               g196(.a(new_n287), .b(new_n271), .c(new_n269), .out0(new_n292));
  nano22aa1n03x5               g197(.a(new_n292), .b(new_n289), .c(new_n290), .out0(new_n293));
  norp02aa1n03x5               g198(.a(new_n291), .b(new_n293), .o1(\s[29] ));
  xorb03aa1n02x5               g199(.a(new_n115), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g200(.a(new_n272), .b(new_n290), .c(new_n277), .out0(new_n296));
  oaih12aa1n02x5               g201(.a(new_n296), .b(new_n283), .c(new_n280), .o1(new_n297));
  oao003aa1n02x5               g202(.a(\a[29] ), .b(\b[28] ), .c(new_n289), .carry(new_n298));
  xnrc02aa1n02x5               g203(.a(\b[29] ), .b(\a[30] ), .out0(new_n299));
  tech160nm_fiaoi012aa1n03p5x5 g204(.a(new_n299), .b(new_n297), .c(new_n298), .o1(new_n300));
  aobi12aa1n02x7               g205(.a(new_n296), .b(new_n271), .c(new_n269), .out0(new_n301));
  nano22aa1n03x5               g206(.a(new_n301), .b(new_n298), .c(new_n299), .out0(new_n302));
  nor002aa1n02x5               g207(.a(new_n300), .b(new_n302), .o1(\s[30] ));
  norb02aa1n02x5               g208(.a(new_n296), .b(new_n299), .out0(new_n304));
  aobi12aa1n02x7               g209(.a(new_n304), .b(new_n271), .c(new_n269), .out0(new_n305));
  oao003aa1n02x5               g210(.a(\a[30] ), .b(\b[29] ), .c(new_n298), .carry(new_n306));
  xnrc02aa1n02x5               g211(.a(\b[30] ), .b(\a[31] ), .out0(new_n307));
  nano22aa1n03x5               g212(.a(new_n305), .b(new_n306), .c(new_n307), .out0(new_n308));
  oaih12aa1n02x5               g213(.a(new_n304), .b(new_n283), .c(new_n280), .o1(new_n309));
  tech160nm_fiaoi012aa1n02p5x5 g214(.a(new_n307), .b(new_n309), .c(new_n306), .o1(new_n310));
  norp02aa1n03x5               g215(.a(new_n310), .b(new_n308), .o1(\s[31] ));
  xnrb03aa1n02x5               g216(.a(new_n117), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g217(.a(\a[3] ), .b(\b[2] ), .c(new_n117), .o1(new_n313));
  xorb03aa1n02x5               g218(.a(new_n313), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g219(.a(new_n119), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  nanp02aa1n02x5               g220(.a(new_n119), .b(new_n122), .o1(new_n316));
  oaoi13aa1n02x5               g221(.a(new_n121), .b(new_n316), .c(\a[5] ), .d(\b[4] ), .o1(new_n317));
  oai112aa1n03x5               g222(.a(new_n316), .b(new_n121), .c(\b[4] ), .d(\a[5] ), .o1(new_n318));
  nanb02aa1n02x5               g223(.a(new_n317), .b(new_n318), .out0(\s[6] ));
  xnbna2aa1n03x5               g224(.a(new_n256), .b(new_n318), .c(new_n257), .out0(\s[7] ));
  aoi013aa1n02x4               g225(.a(new_n103), .b(new_n318), .c(new_n257), .d(new_n104), .o1(new_n321));
  xnrc02aa1n03x5               g226(.a(new_n321), .b(new_n102), .out0(\s[8] ));
  xorb03aa1n02x5               g227(.a(new_n262), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


