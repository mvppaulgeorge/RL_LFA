// Benchmark "adder" written by ABC on Thu Jul 11 12:42:30 2024

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
    new_n125, new_n126, new_n128, new_n129, new_n130, new_n131, new_n132,
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n139, new_n140,
    new_n141, new_n142, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n154, new_n155, new_n156,
    new_n157, new_n159, new_n160, new_n161, new_n162, new_n163, new_n165,
    new_n166, new_n167, new_n168, new_n169, new_n170, new_n172, new_n173,
    new_n174, new_n175, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n187, new_n188,
    new_n189, new_n190, new_n191, new_n192, new_n193, new_n194, new_n196,
    new_n197, new_n198, new_n199, new_n200, new_n201, new_n204, new_n205,
    new_n206, new_n207, new_n208, new_n209, new_n210, new_n211, new_n212,
    new_n213, new_n215, new_n216, new_n217, new_n218, new_n219, new_n220,
    new_n222, new_n223, new_n224, new_n225, new_n226, new_n227, new_n228,
    new_n229, new_n230, new_n231, new_n232, new_n233, new_n235, new_n236,
    new_n237, new_n238, new_n239, new_n240, new_n241, new_n243, new_n244,
    new_n245, new_n246, new_n247, new_n248, new_n249, new_n250, new_n251,
    new_n252, new_n254, new_n255, new_n256, new_n257, new_n258, new_n259,
    new_n260, new_n261, new_n262, new_n263, new_n265, new_n266, new_n267,
    new_n268, new_n269, new_n270, new_n272, new_n273, new_n274, new_n275,
    new_n276, new_n277, new_n278, new_n279, new_n280, new_n282, new_n283,
    new_n284, new_n285, new_n286, new_n287, new_n288, new_n289, new_n290,
    new_n292, new_n293, new_n294, new_n295, new_n296, new_n297, new_n298,
    new_n299, new_n300, new_n303, new_n304, new_n305, new_n306, new_n307,
    new_n308, new_n310, new_n311, new_n312, new_n313, new_n314, new_n315,
    new_n316, new_n319, new_n322, new_n324, new_n326;
  160nm_ficinv00aa1n08x5       g000(.clk(\a[0] ), .clkout(\s[0] ));
  xnrc02aa1n02x5               g001(.a(\b[7] ), .b(\a[8] ), .out0(new_n97));
  160nm_ficinv00aa1n08x5       g002(.clk(\a[8] ), .clkout(new_n98));
  160nm_ficinv00aa1n08x5       g003(.clk(\b[7] ), .clkout(new_n99));
  norp02aa1n02x5               g004(.a(\b[6] ), .b(\a[7] ), .o1(new_n100));
  oaoi03aa1n02x5               g005(.a(new_n98), .b(new_n99), .c(new_n100), .o1(new_n101));
  norp02aa1n02x5               g006(.a(\b[5] ), .b(\a[6] ), .o1(new_n102));
  norp02aa1n02x5               g007(.a(\b[4] ), .b(\a[5] ), .o1(new_n103));
  nanp02aa1n02x5               g008(.a(\b[5] ), .b(\a[6] ), .o1(new_n104));
  aoi012aa1n02x5               g009(.a(new_n102), .b(new_n103), .c(new_n104), .o1(new_n105));
  xnrc02aa1n02x5               g010(.a(\b[6] ), .b(\a[7] ), .out0(new_n106));
  oai013aa1n02x4               g011(.a(new_n101), .b(new_n97), .c(new_n106), .d(new_n105), .o1(new_n107));
  160nm_ficinv00aa1n08x5       g012(.clk(\a[4] ), .clkout(new_n108));
  160nm_ficinv00aa1n08x5       g013(.clk(\b[3] ), .clkout(new_n109));
  nanp02aa1n02x5               g014(.a(new_n109), .b(new_n108), .o1(new_n110));
  nanp02aa1n02x5               g015(.a(\b[3] ), .b(\a[4] ), .o1(new_n111));
  nanp02aa1n02x5               g016(.a(new_n110), .b(new_n111), .o1(new_n112));
  norp02aa1n02x5               g017(.a(\b[2] ), .b(\a[3] ), .o1(new_n113));
  oaoi03aa1n02x5               g018(.a(new_n108), .b(new_n109), .c(new_n113), .o1(new_n114));
  norp02aa1n02x5               g019(.a(\b[1] ), .b(\a[2] ), .o1(new_n115));
  nanp02aa1n02x5               g020(.a(\b[0] ), .b(\a[1] ), .o1(new_n116));
  nanp02aa1n02x5               g021(.a(\b[1] ), .b(\a[2] ), .o1(new_n117));
  aoi012aa1n02x5               g022(.a(new_n115), .b(new_n116), .c(new_n117), .o1(new_n118));
  nanp02aa1n02x5               g023(.a(\b[2] ), .b(\a[3] ), .o1(new_n119));
  nanb02aa1n02x5               g024(.a(new_n113), .b(new_n119), .out0(new_n120));
  oai013aa1n02x4               g025(.a(new_n114), .b(new_n118), .c(new_n120), .d(new_n112), .o1(new_n121));
  nanp02aa1n02x5               g026(.a(\b[4] ), .b(\a[5] ), .o1(new_n122));
  nona23aa1n02x4               g027(.a(new_n104), .b(new_n122), .c(new_n103), .d(new_n102), .out0(new_n123));
  norp03aa1n02x5               g028(.a(new_n123), .b(new_n106), .c(new_n97), .o1(new_n124));
  aoi012aa1n02x5               g029(.a(new_n107), .b(new_n121), .c(new_n124), .o1(new_n125));
  oaoi03aa1n02x5               g030(.a(\a[9] ), .b(\b[8] ), .c(new_n125), .o1(new_n126));
  xorb03aa1n02x5               g031(.a(new_n126), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  norp03aa1n02x5               g032(.a(new_n106), .b(new_n97), .c(new_n105), .o1(new_n128));
  norb02aa1n02x5               g033(.a(new_n101), .b(new_n128), .out0(new_n129));
  nanp02aa1n02x5               g034(.a(new_n121), .b(new_n124), .o1(new_n130));
  norp02aa1n02x5               g035(.a(\b[8] ), .b(\a[9] ), .o1(new_n131));
  norp02aa1n02x5               g036(.a(\b[9] ), .b(\a[10] ), .o1(new_n132));
  nanp02aa1n02x5               g037(.a(\b[9] ), .b(\a[10] ), .o1(new_n133));
  aoi012aa1n02x5               g038(.a(new_n132), .b(new_n131), .c(new_n133), .o1(new_n134));
  nanp02aa1n02x5               g039(.a(\b[8] ), .b(\a[9] ), .o1(new_n135));
  nona23aa1n02x4               g040(.a(new_n133), .b(new_n135), .c(new_n131), .d(new_n132), .out0(new_n136));
  aoai13aa1n02x5               g041(.a(new_n134), .b(new_n136), .c(new_n130), .d(new_n129), .o1(new_n137));
  xorb03aa1n02x5               g042(.a(new_n137), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  160nm_ficinv00aa1n08x5       g043(.clk(\a[12] ), .clkout(new_n139));
  norp02aa1n02x5               g044(.a(\b[10] ), .b(\a[11] ), .o1(new_n140));
  nanp02aa1n02x5               g045(.a(\b[10] ), .b(\a[11] ), .o1(new_n141));
  aoi012aa1n02x5               g046(.a(new_n140), .b(new_n137), .c(new_n141), .o1(new_n142));
  xorb03aa1n02x5               g047(.a(new_n142), .b(\b[11] ), .c(new_n139), .out0(\s[12] ));
  norp02aa1n02x5               g048(.a(\b[11] ), .b(\a[12] ), .o1(new_n144));
  nanp02aa1n02x5               g049(.a(\b[11] ), .b(\a[12] ), .o1(new_n145));
  aoi012aa1n02x5               g050(.a(new_n144), .b(new_n140), .c(new_n145), .o1(new_n146));
  nona23aa1n02x4               g051(.a(new_n145), .b(new_n141), .c(new_n140), .d(new_n144), .out0(new_n147));
  oai012aa1n02x5               g052(.a(new_n146), .b(new_n147), .c(new_n134), .o1(new_n148));
  160nm_ficinv00aa1n08x5       g053(.clk(new_n148), .clkout(new_n149));
  norp02aa1n02x5               g054(.a(new_n147), .b(new_n136), .o1(new_n150));
  aoai13aa1n02x5               g055(.a(new_n150), .b(new_n107), .c(new_n121), .d(new_n124), .o1(new_n151));
  nanp02aa1n02x5               g056(.a(new_n151), .b(new_n149), .o1(new_n152));
  xorb03aa1n02x5               g057(.a(new_n152), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  160nm_ficinv00aa1n08x5       g058(.clk(\a[14] ), .clkout(new_n154));
  norp02aa1n02x5               g059(.a(\b[12] ), .b(\a[13] ), .o1(new_n155));
  nanp02aa1n02x5               g060(.a(\b[12] ), .b(\a[13] ), .o1(new_n156));
  aoi012aa1n02x5               g061(.a(new_n155), .b(new_n152), .c(new_n156), .o1(new_n157));
  xorb03aa1n02x5               g062(.a(new_n157), .b(\b[13] ), .c(new_n154), .out0(\s[14] ));
  norp02aa1n02x5               g063(.a(\b[13] ), .b(\a[14] ), .o1(new_n159));
  nanp02aa1n02x5               g064(.a(\b[13] ), .b(\a[14] ), .o1(new_n160));
  aoi012aa1n02x5               g065(.a(new_n159), .b(new_n155), .c(new_n160), .o1(new_n161));
  nona23aa1n02x4               g066(.a(new_n160), .b(new_n156), .c(new_n155), .d(new_n159), .out0(new_n162));
  aoai13aa1n02x5               g067(.a(new_n161), .b(new_n162), .c(new_n151), .d(new_n149), .o1(new_n163));
  xorb03aa1n02x5               g068(.a(new_n163), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  norp02aa1n02x5               g069(.a(\b[14] ), .b(\a[15] ), .o1(new_n165));
  xorc02aa1n02x5               g070(.a(\a[15] ), .b(\b[14] ), .out0(new_n166));
  xnrc02aa1n02x5               g071(.a(\b[15] ), .b(\a[16] ), .out0(new_n167));
  aoai13aa1n02x5               g072(.a(new_n167), .b(new_n165), .c(new_n163), .d(new_n166), .o1(new_n168));
  nanp02aa1n02x5               g073(.a(new_n163), .b(new_n166), .o1(new_n169));
  nona22aa1n02x4               g074(.a(new_n169), .b(new_n167), .c(new_n165), .out0(new_n170));
  nanp02aa1n02x5               g075(.a(new_n170), .b(new_n168), .o1(\s[16] ));
  xnrc02aa1n02x5               g076(.a(\b[14] ), .b(\a[15] ), .out0(new_n172));
  160nm_ficinv00aa1n08x5       g077(.clk(new_n165), .clkout(new_n173));
  oao003aa1n02x5               g078(.a(\a[16] ), .b(\b[15] ), .c(new_n173), .carry(new_n174));
  oai013aa1n02x4               g079(.a(new_n174), .b(new_n167), .c(new_n172), .d(new_n161), .o1(new_n175));
  norp03aa1n02x5               g080(.a(new_n162), .b(new_n167), .c(new_n172), .o1(new_n176));
  aoi012aa1n02x5               g081(.a(new_n175), .b(new_n148), .c(new_n176), .o1(new_n177));
  nano23aa1n02x4               g082(.a(new_n131), .b(new_n132), .c(new_n133), .d(new_n135), .out0(new_n178));
  nano23aa1n02x4               g083(.a(new_n140), .b(new_n144), .c(new_n145), .d(new_n141), .out0(new_n179));
  nano23aa1n02x4               g084(.a(new_n155), .b(new_n159), .c(new_n160), .d(new_n156), .out0(new_n180));
  xorc02aa1n02x5               g085(.a(\a[16] ), .b(\b[15] ), .out0(new_n181));
  nanp03aa1n02x5               g086(.a(new_n180), .b(new_n166), .c(new_n181), .o1(new_n182));
  nano22aa1n02x4               g087(.a(new_n182), .b(new_n178), .c(new_n179), .out0(new_n183));
  aoai13aa1n02x5               g088(.a(new_n183), .b(new_n107), .c(new_n121), .d(new_n124), .o1(new_n184));
  xorc02aa1n02x5               g089(.a(\a[17] ), .b(\b[16] ), .out0(new_n185));
  xnbna2aa1n03x5               g090(.a(new_n185), .b(new_n184), .c(new_n177), .out0(\s[17] ));
  nanp02aa1n02x5               g091(.a(new_n176), .b(new_n150), .o1(new_n187));
  aoai13aa1n02x5               g092(.a(new_n177), .b(new_n187), .c(new_n130), .d(new_n129), .o1(new_n188));
  norp02aa1n02x5               g093(.a(\b[16] ), .b(\a[17] ), .o1(new_n189));
  aoi012aa1n02x5               g094(.a(new_n189), .b(new_n188), .c(new_n185), .o1(new_n190));
  160nm_ficinv00aa1n08x5       g095(.clk(\a[18] ), .clkout(new_n191));
  160nm_ficinv00aa1n08x5       g096(.clk(\b[17] ), .clkout(new_n192));
  nanp02aa1n02x5               g097(.a(new_n192), .b(new_n191), .o1(new_n193));
  nanp02aa1n02x5               g098(.a(\b[17] ), .b(\a[18] ), .o1(new_n194));
  xnbna2aa1n03x5               g099(.a(new_n190), .b(new_n194), .c(new_n193), .out0(\s[18] ));
  aob012aa1n02x5               g100(.a(new_n193), .b(new_n189), .c(new_n194), .out0(new_n196));
  160nm_ficinv00aa1n08x5       g101(.clk(new_n196), .clkout(new_n197));
  160nm_ficinv00aa1n08x5       g102(.clk(new_n185), .clkout(new_n198));
  nano22aa1n02x4               g103(.a(new_n198), .b(new_n193), .c(new_n194), .out0(new_n199));
  160nm_ficinv00aa1n08x5       g104(.clk(new_n199), .clkout(new_n200));
  aoai13aa1n02x5               g105(.a(new_n197), .b(new_n200), .c(new_n184), .d(new_n177), .o1(new_n201));
  xorb03aa1n02x5               g106(.a(new_n201), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g107(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  norp02aa1n02x5               g108(.a(\b[18] ), .b(\a[19] ), .o1(new_n204));
  nanp02aa1n02x5               g109(.a(\b[18] ), .b(\a[19] ), .o1(new_n205));
  norp02aa1n02x5               g110(.a(\b[19] ), .b(\a[20] ), .o1(new_n206));
  nanp02aa1n02x5               g111(.a(\b[19] ), .b(\a[20] ), .o1(new_n207));
  norb02aa1n02x5               g112(.a(new_n207), .b(new_n206), .out0(new_n208));
  160nm_ficinv00aa1n08x5       g113(.clk(new_n208), .clkout(new_n209));
  aoai13aa1n02x5               g114(.a(new_n209), .b(new_n204), .c(new_n201), .d(new_n205), .o1(new_n210));
  norb02aa1n02x5               g115(.a(new_n205), .b(new_n204), .out0(new_n211));
  aoai13aa1n02x5               g116(.a(new_n211), .b(new_n196), .c(new_n188), .d(new_n199), .o1(new_n212));
  nona22aa1n02x4               g117(.a(new_n212), .b(new_n209), .c(new_n204), .out0(new_n213));
  nanp02aa1n02x5               g118(.a(new_n210), .b(new_n213), .o1(\s[20] ));
  aoi012aa1n02x5               g119(.a(new_n206), .b(new_n204), .c(new_n207), .o1(new_n215));
  160nm_ficinv00aa1n08x5       g120(.clk(new_n215), .clkout(new_n216));
  nano23aa1n02x4               g121(.a(new_n204), .b(new_n206), .c(new_n207), .d(new_n205), .out0(new_n217));
  aoi012aa1n02x5               g122(.a(new_n216), .b(new_n217), .c(new_n196), .o1(new_n218));
  nanp02aa1n02x5               g123(.a(new_n199), .b(new_n217), .o1(new_n219));
  aoai13aa1n02x5               g124(.a(new_n218), .b(new_n219), .c(new_n184), .d(new_n177), .o1(new_n220));
  xorb03aa1n02x5               g125(.a(new_n220), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  norp02aa1n02x5               g126(.a(\b[20] ), .b(\a[21] ), .o1(new_n222));
  nanp02aa1n02x5               g127(.a(\b[20] ), .b(\a[21] ), .o1(new_n223));
  norb02aa1n02x5               g128(.a(new_n223), .b(new_n222), .out0(new_n224));
  norp02aa1n02x5               g129(.a(\b[21] ), .b(\a[22] ), .o1(new_n225));
  nanp02aa1n02x5               g130(.a(\b[21] ), .b(\a[22] ), .o1(new_n226));
  norb02aa1n02x5               g131(.a(new_n226), .b(new_n225), .out0(new_n227));
  160nm_ficinv00aa1n08x5       g132(.clk(new_n227), .clkout(new_n228));
  aoai13aa1n02x5               g133(.a(new_n228), .b(new_n222), .c(new_n220), .d(new_n224), .o1(new_n229));
  160nm_ficinv00aa1n08x5       g134(.clk(new_n218), .clkout(new_n230));
  160nm_ficinv00aa1n08x5       g135(.clk(new_n219), .clkout(new_n231));
  aoai13aa1n02x5               g136(.a(new_n224), .b(new_n230), .c(new_n188), .d(new_n231), .o1(new_n232));
  nona22aa1n02x4               g137(.a(new_n232), .b(new_n228), .c(new_n222), .out0(new_n233));
  nanp02aa1n02x5               g138(.a(new_n229), .b(new_n233), .o1(\s[22] ));
  aoi012aa1n02x5               g139(.a(new_n225), .b(new_n222), .c(new_n226), .o1(new_n235));
  160nm_ficinv00aa1n08x5       g140(.clk(new_n235), .clkout(new_n236));
  nano23aa1n02x4               g141(.a(new_n222), .b(new_n225), .c(new_n226), .d(new_n223), .out0(new_n237));
  aoi012aa1n02x5               g142(.a(new_n236), .b(new_n230), .c(new_n237), .o1(new_n238));
  nano22aa1n02x4               g143(.a(new_n200), .b(new_n217), .c(new_n237), .out0(new_n239));
  160nm_ficinv00aa1n08x5       g144(.clk(new_n239), .clkout(new_n240));
  aoai13aa1n02x5               g145(.a(new_n238), .b(new_n240), .c(new_n184), .d(new_n177), .o1(new_n241));
  xorb03aa1n02x5               g146(.a(new_n241), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g147(.a(\b[22] ), .b(\a[23] ), .o1(new_n243));
  nanp02aa1n02x5               g148(.a(\b[22] ), .b(\a[23] ), .o1(new_n244));
  norb02aa1n02x5               g149(.a(new_n244), .b(new_n243), .out0(new_n245));
  norp02aa1n02x5               g150(.a(\b[23] ), .b(\a[24] ), .o1(new_n246));
  nanp02aa1n02x5               g151(.a(\b[23] ), .b(\a[24] ), .o1(new_n247));
  nanb02aa1n02x5               g152(.a(new_n246), .b(new_n247), .out0(new_n248));
  aoai13aa1n02x5               g153(.a(new_n248), .b(new_n243), .c(new_n241), .d(new_n245), .o1(new_n249));
  160nm_ficinv00aa1n08x5       g154(.clk(new_n238), .clkout(new_n250));
  aoai13aa1n02x5               g155(.a(new_n245), .b(new_n250), .c(new_n188), .d(new_n239), .o1(new_n251));
  nona22aa1n02x4               g156(.a(new_n251), .b(new_n248), .c(new_n243), .out0(new_n252));
  nanp02aa1n02x5               g157(.a(new_n249), .b(new_n252), .o1(\s[24] ));
  nanp03aa1n02x5               g158(.a(new_n196), .b(new_n211), .c(new_n208), .o1(new_n254));
  160nm_fiao0012aa1n02p5x5     g159(.a(new_n246), .b(new_n243), .c(new_n247), .o(new_n255));
  nano23aa1n02x4               g160(.a(new_n243), .b(new_n246), .c(new_n247), .d(new_n244), .out0(new_n256));
  aoi012aa1n02x5               g161(.a(new_n255), .b(new_n256), .c(new_n236), .o1(new_n257));
  nanp02aa1n02x5               g162(.a(new_n256), .b(new_n237), .o1(new_n258));
  aoai13aa1n02x5               g163(.a(new_n257), .b(new_n258), .c(new_n254), .d(new_n215), .o1(new_n259));
  160nm_ficinv00aa1n08x5       g164(.clk(new_n259), .clkout(new_n260));
  nano32aa1n02x4               g165(.a(new_n200), .b(new_n256), .c(new_n217), .d(new_n237), .out0(new_n261));
  160nm_ficinv00aa1n08x5       g166(.clk(new_n261), .clkout(new_n262));
  aoai13aa1n02x5               g167(.a(new_n260), .b(new_n262), .c(new_n184), .d(new_n177), .o1(new_n263));
  xorb03aa1n02x5               g168(.a(new_n263), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g169(.a(\b[24] ), .b(\a[25] ), .o1(new_n265));
  xorc02aa1n02x5               g170(.a(\a[25] ), .b(\b[24] ), .out0(new_n266));
  xnrc02aa1n02x5               g171(.a(\b[25] ), .b(\a[26] ), .out0(new_n267));
  aoai13aa1n02x5               g172(.a(new_n267), .b(new_n265), .c(new_n263), .d(new_n266), .o1(new_n268));
  aoai13aa1n02x5               g173(.a(new_n266), .b(new_n259), .c(new_n188), .d(new_n261), .o1(new_n269));
  nona22aa1n02x4               g174(.a(new_n269), .b(new_n267), .c(new_n265), .out0(new_n270));
  nanp02aa1n02x5               g175(.a(new_n268), .b(new_n270), .o1(\s[26] ));
  160nm_ficinv00aa1n08x5       g176(.clk(\a[26] ), .clkout(new_n272));
  160nm_ficinv00aa1n08x5       g177(.clk(\b[25] ), .clkout(new_n273));
  oaoi03aa1n02x5               g178(.a(new_n272), .b(new_n273), .c(new_n265), .o1(new_n274));
  160nm_ficinv00aa1n08x5       g179(.clk(new_n274), .clkout(new_n275));
  norb02aa1n02x5               g180(.a(new_n266), .b(new_n267), .out0(new_n276));
  aoi012aa1n02x5               g181(.a(new_n275), .b(new_n259), .c(new_n276), .o1(new_n277));
  nanb02aa1n02x5               g182(.a(new_n267), .b(new_n266), .out0(new_n278));
  nona22aa1n02x4               g183(.a(new_n231), .b(new_n258), .c(new_n278), .out0(new_n279));
  aoai13aa1n02x5               g184(.a(new_n277), .b(new_n279), .c(new_n184), .d(new_n177), .o1(new_n280));
  xorb03aa1n02x5               g185(.a(new_n280), .b(\b[26] ), .c(\a[27] ), .out0(\s[27] ));
  norp02aa1n02x5               g186(.a(\b[26] ), .b(\a[27] ), .o1(new_n282));
  xorc02aa1n02x5               g187(.a(\a[27] ), .b(\b[26] ), .out0(new_n283));
  xnrc02aa1n02x5               g188(.a(\b[27] ), .b(\a[28] ), .out0(new_n284));
  aoai13aa1n02x5               g189(.a(new_n284), .b(new_n282), .c(new_n280), .d(new_n283), .o1(new_n285));
  nanp02aa1n02x5               g190(.a(new_n259), .b(new_n276), .o1(new_n286));
  nanp02aa1n02x5               g191(.a(new_n286), .b(new_n274), .o1(new_n287));
  nano32aa1n02x4               g192(.a(new_n219), .b(new_n276), .c(new_n237), .d(new_n256), .out0(new_n288));
  aoai13aa1n02x5               g193(.a(new_n283), .b(new_n287), .c(new_n188), .d(new_n288), .o1(new_n289));
  nona22aa1n02x4               g194(.a(new_n289), .b(new_n284), .c(new_n282), .out0(new_n290));
  nanp02aa1n02x5               g195(.a(new_n285), .b(new_n290), .o1(\s[28] ));
  160nm_ficinv00aa1n08x5       g196(.clk(\a[28] ), .clkout(new_n292));
  160nm_ficinv00aa1n08x5       g197(.clk(\b[27] ), .clkout(new_n293));
  oaoi03aa1n02x5               g198(.a(new_n292), .b(new_n293), .c(new_n282), .o1(new_n294));
  160nm_ficinv00aa1n08x5       g199(.clk(new_n294), .clkout(new_n295));
  norb02aa1n02x5               g200(.a(new_n283), .b(new_n284), .out0(new_n296));
  aoai13aa1n02x5               g201(.a(new_n296), .b(new_n287), .c(new_n188), .d(new_n288), .o1(new_n297));
  xnrc02aa1n02x5               g202(.a(\b[28] ), .b(\a[29] ), .out0(new_n298));
  nona22aa1n02x4               g203(.a(new_n297), .b(new_n298), .c(new_n295), .out0(new_n299));
  aoai13aa1n02x5               g204(.a(new_n298), .b(new_n295), .c(new_n280), .d(new_n296), .o1(new_n300));
  nanp02aa1n02x5               g205(.a(new_n300), .b(new_n299), .o1(\s[29] ));
  xorb03aa1n02x5               g206(.a(new_n116), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  oaoi03aa1n02x5               g207(.a(\a[29] ), .b(\b[28] ), .c(new_n294), .o1(new_n303));
  norb03aa1n02x5               g208(.a(new_n283), .b(new_n298), .c(new_n284), .out0(new_n304));
  xnrc02aa1n02x5               g209(.a(\b[29] ), .b(\a[30] ), .out0(new_n305));
  aoai13aa1n02x5               g210(.a(new_n305), .b(new_n303), .c(new_n280), .d(new_n304), .o1(new_n306));
  aoai13aa1n02x5               g211(.a(new_n304), .b(new_n287), .c(new_n188), .d(new_n288), .o1(new_n307));
  nona22aa1n02x4               g212(.a(new_n307), .b(new_n305), .c(new_n303), .out0(new_n308));
  nanp02aa1n02x5               g213(.a(new_n306), .b(new_n308), .o1(\s[30] ));
  nanb02aa1n02x5               g214(.a(new_n305), .b(new_n303), .out0(new_n310));
  oai012aa1n02x5               g215(.a(new_n310), .b(\b[29] ), .c(\a[30] ), .o1(new_n311));
  norb02aa1n02x5               g216(.a(new_n304), .b(new_n305), .out0(new_n312));
  aoai13aa1n02x5               g217(.a(new_n312), .b(new_n287), .c(new_n188), .d(new_n288), .o1(new_n313));
  xnrc02aa1n02x5               g218(.a(\b[30] ), .b(\a[31] ), .out0(new_n314));
  nona22aa1n02x4               g219(.a(new_n313), .b(new_n314), .c(new_n311), .out0(new_n315));
  aoai13aa1n02x5               g220(.a(new_n314), .b(new_n311), .c(new_n280), .d(new_n312), .o1(new_n316));
  nanp02aa1n02x5               g221(.a(new_n316), .b(new_n315), .o1(\s[31] ));
  xnrb03aa1n02x5               g222(.a(new_n118), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g223(.a(\a[3] ), .b(\b[2] ), .c(new_n118), .o1(new_n319));
  xorb03aa1n02x5               g224(.a(new_n319), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g225(.a(new_n121), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  aoi012aa1n02x5               g226(.a(new_n103), .b(new_n121), .c(new_n122), .o1(new_n322));
  xnrb03aa1n02x5               g227(.a(new_n322), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  oaib12aa1n02x5               g228(.a(new_n105), .b(new_n123), .c(new_n121), .out0(new_n324));
  xorb03aa1n02x5               g229(.a(new_n324), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  aoib12aa1n02x5               g230(.a(new_n100), .b(new_n324), .c(new_n106), .out0(new_n326));
  xorb03aa1n02x5               g231(.a(new_n326), .b(\b[7] ), .c(new_n98), .out0(\s[8] ));
  xnrb03aa1n02x5               g232(.a(new_n125), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


